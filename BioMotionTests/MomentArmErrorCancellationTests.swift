import XCTest
@testable import BioMotion

/// **Does a wrong moment arm cancel out of a LEFT/RIGHT comparison, and does it
/// fail to cancel BETWEEN muscles?** Measured on the shipping QP, not argued.
///
/// # Why this file decides what the screen may say
///
/// `MomentArmComputer` logs at load:
///
///     ⚠ 76 PathWrap references are NOT modelled. Those muscles take a
///     straight-line shortcut where the real path wraps around bone, so their
///     L_MT and moment arms are wrong (worst at flexed poses, where the sign
///     can flip)
///
/// Parsing the shipped `FullBody.osim` says which 66 muscles those are, and
/// `MomentArmTests.testTheUnmodelledWrapTableMatchesTheShippedModels` pins the
/// list against the parser's own report: it contains glmax1/2/3, recfem,
/// vasmed, vaslat140, vasint, gasmed, gaslat140, semimem, semiten, bfsh140,
/// psoas, iliacus, addlong, addbrev, addmag{Prox,Mid,Dist,Isch} and grac —
/// essentially every lower-limb muscle a runner would recognise. Running is a
/// flexed-pose activity, so this is not a corner case for this product.
///
/// A wrong moment arm `r → k·r` maps a joint torque to a wrong muscle force
/// `F = τ/r`, hence a wrong activation. The claim under test is that the error
/// is a PER-MUSCLE SCALE, so:
///
/// * it cancels in `(L − R)/mean` for that muscle, because the same wrong path
///   is used on both sides — this is the deliverable and it survives;
/// * it does NOT cancel between two different muscles, because their errors
///   differ — so a muscle-to-muscle ordering is not a measurement.
///
/// The QP couples every muscle through `min ½aᵀPa + ½λ‖A a − τ‖²`, so "it is
/// just a scale" is not obvious: perturbing one muscle's column of `A`
/// redistributes load across all of them. That redistribution is what these
/// tests measure.
///
/// # The regime the argument holds in, and the two places it stops
///
/// The QP is linear in `τ` — and therefore the left/right ratio is exactly
/// invariant to a per-muscle scale on `A` — only while NO activation sits on a
/// bound. Both bounds are reachable and both are measured here:
/// `a ≤ 1` (`MuscleLoad.isSaturated`) and `a ≥ aMin = 0.02`
/// (`MuscleLoad.isAtActivationFloor`). The sign-flip test is what found the
/// second one: a muscle whose modelled path has the wrong sign is pushed onto
/// the floor on BOTH sides and reads exactly 0 % left/right, i.e. a lost finding
/// presented as an even one. That is why the floor now withholds too.
///
/// # The numerical floor on any of these figures
///
/// OSQP stops on `eps_abs = eps_rel = 1e-3` with polishing off, so an activation
/// carries about 2e-3 of absolute slack and a left/right percentage built from
/// two of them carries a couple of percentage points. The unperturbed rig is
/// solved for exactly that reason: its analytic answer is known, so the gap
/// between the analytic answer and what OSQP returns IS the noise floor, and the
/// perturbation tests are calibrated against it rather than against a guess.
///
/// # The load-bearing assumption, isolated deliberately
///
/// The cancellation needs the error to be the SAME factor on both sides.
/// `testAOneSidedMomentArmErrorDoesNotCancelAndIsTheAssumptionTheClaimRestsOn`
/// perturbs one side only and shows the left/right figure moves by an order of
/// magnitude more, which is the falsifier: if the two legs were ever modelled
/// with different paths, or sampled at poses far enough apart that the same
/// wrap error evaluates differently, the left/right claim would go too.
final class MomentArmErrorCancellationTests: XCTestCase {

    // MARK: - The rig

    /// A bilateral rig with no shared DOF between the sides — which is what the
    /// real model is: no muscle in `FullBody.osim` spans the left and right
    /// legs. Two DOFs and three muscles per side, one of which spans both DOFs,
    /// so the QP has real redundancy to redistribute through.
    /// The rig's analytic answer: with every right torque `rightScale` times its
    /// left counterpart and no bound active, the QP's solution is linear in `τ`,
    /// so EVERY muscle reads the same figure.
    static var analyticDifferencePercent: Double {
        100 * (1 - Rig.rightScale) / (0.5 * (1 + Rig.rightScale))
    }

    /// How far the unperturbed solve lands from that analytic answer — i.e. what
    /// OSQP's own termination tolerance is worth in these units.
    private func solverNoiseFloor() throws -> Double {
        let a = try solve()
        var worst = 0.0
        for (base, _, _, _) in Rig.muscles {
            let d = try differencePercent(a, base)
            worst = Swift.max(worst, abs(d - Self.analyticDifferencePercent))
        }
        return worst
    }

    private enum Rig {
        static let dofNames = ["hip_l", "knee_l", "hip_r", "knee_r"]
        /// base name → (hip arm, knee arm, F_max)
        static let muscles: [(base: String, hip: Double, knee: Double, fmax: Double)] = [
            ("alpha", 0.050, 0.000, 3000),
            ("beta",  0.000, 0.040, 2000),
            ("gamma", 0.030, 0.050, 2500),
        ]
        static let optimalFiberLength = 0.1
        static let tendonSlackLength = 0.2
        /// `l_opt + l_Ts`: normalised fibre length 1, so `f_AL = 1` and the
        /// force scale is exactly `F_max`.
        static let isometricLength = 0.3
        static let softPenalty = 100.0

        /// A genuinely asymmetric runner: every RIGHT torque is `rightScale`
        /// times its left counterpart. The QP is linear in `τ` while no bound is
        /// active, so the true per-muscle answer is a uniform
        /// `100·(1 − rightScale)/(0.5·(1 + rightScale))` on EVERY muscle — a
        /// number this file checks rather than assumes.
        static let leftTorques: [String: Double] = ["hip_l": 135, "knee_l": 90]
        static let rightScale = 0.8
    }

    /// One solve on a FRESH solver, so warm starts and `L_MT` history cannot
    /// couple two cases together.
    ///
    /// - Parameter scale: per-muscle-base multiplier on BOTH sides' moment arms.
    /// - Parameter leftOnlyScale: per-muscle-base multiplier applied to the LEFT
    ///   side only, on top of `scale`.
    private func solve(scale: [String: Double] = [:],
                       leftOnlyScale: [String: Double] = [:]) throws -> [String: Double] {
        var names: [String] = []
        var arms: [Double] = []      // row-major [nMuscles x nDOFs]
        var maxForces: [Double] = []
        for (base, hip, knee, fmax) in Rig.muscles {
            for side in ["l", "r"] {
                let k = (scale[base] ?? 1) * (side == "l" ? (leftOnlyScale[base] ?? 1) : 1)
                names.append("\(base)_\(side)")
                maxForces.append(fmax)
                // Columns are ordered hip_l, knee_l, hip_r, knee_r. A muscle
                // acts on its own side only.
                arms.append(contentsOf: side == "l"
                            ? [k * hip, k * knee, 0, 0]
                            : [0, 0, k * hip, k * knee])
            }
        }
        let torques = Rig.dofNames.map { dof -> Double in
            if let t = Rig.leftTorques[dof] { return t }
            let mirrored = dof.replacingOccurrences(of: "_r", with: "_l")
            return (Rig.leftTorques[mirrored] ?? 0) * Rig.rightScale
        }
        let result = MuscleSolver().solveReal(
            withJointTorques: torques.map(NSNumber.init(value:)),
            momentArms: arms.map(NSNumber.init(value:)),
            muscleNames: names,
            muscleLengths: names.map { _ in NSNumber(value: Rig.isometricLength) },
            maxForces: maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: names.map { _ in NSNumber(value: Rig.optimalFiberLength) },
            tendonSlackLengths: names.map { _ in NSNumber(value: Rig.tendonSlackLength) },
            pennationAngles: names.map { _ in NSNumber(value: 0.0) },
            jointVelocities: Rig.dofNames.map { _ in NSNumber(value: 0.0) },
            dofNames: Rig.dofNames,
            dt: 1.0 / 30.0,
            softPenalty: Rig.softPenalty)
        let unwrapped = try XCTUnwrap(result, "the QP returned nothing")
        XCTAssertTrue(unwrapped.converged)
        var out: [String: Double] = [:]
        for (i, name) in unwrapped.muscleNames.enumerated() {
            out[name] = unwrapped.activations[i].doubleValue
        }
        return out
    }

    /// `100·(L − R)/mean` — the panel's own statistic.
    private func differencePercent(_ a: [String: Double], _ base: String) throws -> Double {
        let l = try XCTUnwrap(a["\(base)_l"]), r = try XCTUnwrap(a["\(base)_r"])
        return 100 * (l - r) / (0.5 * (l + r))
    }

    /// The heavier side's activation, which is exactly what the panel's ranking
    /// key used to be.
    private func rankingKey(_ a: [String: Double], _ base: String) throws -> Double {
        Swift.max(try XCTUnwrap(a["\(base)_l"]), try XCTUnwrap(a["\(base)_r"]))
    }

    // MARK: - Preconditions, checked rather than assumed

    /// The linearity the whole argument rests on holds only while no activation
    /// sits on a bound. If one did, the QP would be piecewise linear and a
    /// scale would stop cancelling — which is precisely what
    /// `MuscleLoad.isSaturated` is for. Checked here so the two tests below
    /// cannot pass for the wrong reason.
    func testNoActivationSitsOnABoundInThisRig() throws {
        for activations in [try solve(), try solve(scale: ["gamma": 0.6])] {
            for (name, a) in activations {
                XCTAssertGreaterThan(a, MuscleSolver().minActivation + 1e-3,
                                     "\(name) is on the lower bound")
                XCTAssertLessThan(a, 1 - 1e-2, "\(name) is on the upper bound")
            }
        }
    }

    /// The rig really does carry a left/right difference, so "the difference did
    /// not move" below is not the trivial statement that it was zero.
    func testTheUnperturbedRigCarriesTheAsymmetryItWasBuiltWith() throws {
        let a = try solve()
        let expected = Self.analyticDifferencePercent
        var measured: [Double] = []
        for (base, _, _, _) in Rig.muscles {
            let d = try differencePercent(a, base)
            measured.append(d)
            // 2 percentage points is the SOLVER's slack, not a tolerance chosen
            // to make this pass: `solverNoiseFloor` below is the measured gap and
            // every other assertion in this file is scaled by it.
            XCTAssertEqual(d, expected, accuracy: 2.0,
                           "\(base) should read the rig's \(expected) % left-high")
        }
        print("MOMENT-ARM-METRIC baseline_difference_percent=\(expected) measured=\(measured) "
              + "activations=\(a.sorted { $0.key < $1.key }.map { "\($0.key)=\($0.value)" })")
    }

    // MARK: - The finding

    /// **A per-muscle moment-arm error cancels out of that muscle's left/right
    /// comparison and does NOT cancel out of the muscle-to-muscle ordering.**
    ///
    /// This is the measurement the panel's scoping decision rests on: the
    /// left/right sentence stays, the cross-muscle ranking goes.
    func testABilateralMomentArmErrorLeavesLeftRightAloneAndMovesTheRanking() throws {
        let truth = try solve()
        // A straight-line shortcut where the path really wraps around bone.
        // 0.6 is well inside what a missing wrap costs — the log's own warning
        // says the SIGN can flip at flexed poses, which is a scale of −1.
        let wrong = try solve(scale: ["gamma": 0.6])

        var worstDifferenceShift = 0.0
        var worstRankingShiftPercent = 0.0
        var truthKeys: [String: Double] = [:]
        var wrongKeys: [String: Double] = [:]
        for (base, _, _, _) in Rig.muscles {
            let dTruth = try differencePercent(truth, base)
            let dWrong = try differencePercent(wrong, base)
            worstDifferenceShift = Swift.max(worstDifferenceShift, abs(dWrong - dTruth))
            // The ranking key is an ABSOLUTE activation, so it moves — both for
            // the perturbed muscle and, through the QP's redistribution of the
            // torque it no longer supplies, for the others.
            let t = try rankingKey(truth, base)
            let w = try rankingKey(wrong, base)
            truthKeys[base] = t
            wrongKeys[base] = w
            worstRankingShiftPercent = Swift.max(worstRankingShiftPercent, 100 * abs(w - t) / t)
        }

        // And the ORDER itself, which is what the user reads.
        let bases = Rig.muscles.map(\.base)
        let truthOrder = bases.sorted { (truthKeys[$0] ?? 0) > (truthKeys[$1] ?? 0) }
        let wrongOrder = bases.sorted { (wrongKeys[$0] ?? 0) > (wrongKeys[$1] ?? 0) }

        let noise = try solverNoiseFloor()
        print("MOMENT-ARM-METRIC bilateral_perturbation gamma_scale=0.6 "
              + "worst_left_right_shift_pp=\(worstDifferenceShift) "
              + "solver_noise_floor_pp=\(noise) "
              + "worst_ranking_shift_percent=\(worstRankingShiftPercent) "
              + "order_truth=\(truthOrder) order_wrong=\(wrongOrder)")

        // The left/right figure must not move by more than the solver's own
        // slack already moves it. That is the strongest statement available: the
        // analytic answer says the shift is exactly zero, and OSQP cannot
        // resolve zero any finer than `noise`.
        XCTAssertLessThanOrEqual(worstDifferenceShift, 2 * noise,
                                 "a bilateral moment-arm error must not move a left/right figure "
                                 + "by more than the solver's own tolerance does "
                                 + "(shift \(worstDifferenceShift) pp, noise \(noise) pp)")
        XCTAssertLessThan(worstDifferenceShift, 2.0, "and it must stay small in absolute terms")
        XCTAssertGreaterThan(worstRankingShiftPercent, 10 * worstDifferenceShift,
                             "and it must move the cross-muscle quantity by far more, or the "
                             + "distinction this panel is scoped on does not exist")
        XCTAssertNotEqual(truthOrder, wrongOrder,
                          "the ranking the panel used to publish must be shown to reorder")
    }

    /// **The sign flip the loader's own warning names, and what it actually
    /// does.**
    ///
    /// Not what this file first assumed. A muscle whose modelled moment arm has
    /// the wrong sign is not merely rescaled — the QP refuses to recruit it and
    /// pushes it onto the `a ≥ aMin` bound on BOTH sides, where it reads exactly
    /// 0 % left/right. That is a real finding turned into "even", and it is why
    /// `MuscleLoad.isAtActivationFloor` exists and withholds: the cancellation
    /// argument holds in the interior of the box and nowhere else.
    func testASignFlippedMomentArmPinsTheMuscleToTheFloorAndReadsFalselyEven() throws {
        let truth = try solve()
        let flipped = try solve(scale: ["gamma": -1.0])
        let floor = GaitLoadSummary.activationFloorThreshold
        let left = try XCTUnwrap(flipped["gamma_l"])
        let right = try XCTUnwrap(flipped["gamma_r"])
        let dFlipped = try differencePercent(flipped, "gamma")
        let dTruth = try differencePercent(truth, "gamma")
        print("MOMENT-ARM-METRIC sign_flip gamma_l=\(left) gamma_r=\(right) "
              + "floor_threshold=\(floor) truth_percent=\(dTruth) flipped_percent=\(dFlipped)")

        XCTAssertLessThanOrEqual(left, floor, "a wrong-signed muscle is not recruited")
        XCTAssertLessThanOrEqual(right, floor)
        XCTAssertEqual(dFlipped, 0, accuracy: 1.0,
                       "and both sides land on the same bound, so it reads EVEN")
        XCTAssertGreaterThan(abs(dTruth), 10.0,
                             "while the truth is a large real difference — this is a finding "
                             + "destroyed, not a false positive")
        // Which is exactly the case the summary now withholds instead of
        // publishing as "even to within what this clip can resolve".
        XCTAssertTrue(GaitLoadSummary.activationFloorThreshold > MuscleSolver().minActivation)
    }

    /// **The assumption, isolated.** If the two sides' paths were modelled
    /// differently — or the same wrap error evaluated differently at the two
    /// legs' sampled poses — the cancellation would not hold, and this is how
    /// much it would cost. Reported so the residual risk in the shipped claim
    /// has a number attached rather than a shrug.
    func testAOneSidedMomentArmErrorDoesNotCancelAndIsTheAssumptionTheClaimRestsOn() throws {
        let truth = try solve()
        let oneSided = try solve(leftOnlyScale: ["gamma": 0.6])
        let dOneSided = try differencePercent(oneSided, "gamma")
        let dTruth = try differencePercent(truth, "gamma")
        let shift = abs(dOneSided - dTruth)
        print("MOMENT-ARM-METRIC one_sided_perturbation gamma_scale_left_only=0.6 "
              + "left_right_shift_pp=\(shift)")
        XCTAssertGreaterThan(shift, 10.0,
                             "a one-sided error must move the left/right figure — the claim "
                             + "depends on the model being bilaterally identical")
    }
}
