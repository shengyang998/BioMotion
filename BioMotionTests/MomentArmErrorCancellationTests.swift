import XCTest
@testable import BioMotion

/// **Does a wrong moment arm cancel out of a LEFT/RIGHT comparison?** Measured
/// on the shipping QP, not argued. As of 2026-08-08 the answer is NO, and this
/// file is the measurement that retired the product's last per-muscle claim.
///
/// # The experiment that could not fail, and what replaced it
///
/// This file used to answer YES, on a rig where every RIGHT joint torque was
/// `0.8 ×` its left counterpart. The shipped QP is
///
///     min ½ aᵀ(εI + λAᵀA) a − λ τᵀA a,   a ∈ [0.02, 1]
///
/// whose interior solution is `a = (εI + λAᵀA)⁻¹ λAᵀ τ` — **linear in `τ`**. So
/// under `τ_R = c·τ_L` the solution obeys `a_R = c·a_L` EXACTLY, for any moment
/// arm matrix whatsoever. The perturbation could not move the answer: in exact
/// arithmetic the "measured" shift is 1e-6 pp, and the 1.04 pp that was reported
/// as evidence of cancellation is OSQP's own termination tolerance.
/// `testTheProportionalRigIsAnIdentityAndCertifiesNothing` keeps that case and
/// says so, because the identity is worth stating — it is half the argument for
/// retirement.
///
/// The other half is the case that CAN fail:
/// `testAShapeAsymmetryMakesABilateralMomentArmErrorLeak` gives the right leg a
/// different torque PATTERN rather than a different size — 0.80× the left's hip
/// moment, 1.00× its knee moment, which is what a gait asymmetry is — and
/// measures the same bilateral `×0.6` moment-arm error moving a published
/// left/right figure by more than any of the three pinned clips can resolve.
///
/// # Why the two halves together end the claim rather than bounding it
///
/// The leak vanishes exactly when the two legs' torques are proportional. But in
/// that regime every muscle returns the SAME left/right figure — the linearity
/// again — so the per-muscle breakdown carries no per-muscle information at all.
/// Every bit of per-muscle differentiation in the statistic comes from the
/// non-proportional part of the torque, and that is precisely the part a wrong
/// moment arm distorts. There is no regime where the claim is both safe and
/// informative, which is why `GaitLoadSummary.perMuscleLeftRightClaimIsSupported`
/// is a flat `false` rather than a gate.
///
/// # The two other bounds, both still measured here
///
/// * A ONE-SIDED error (different paths on the two legs, or the same wrap error
///   evaluated at two different poses) costs more than 10 pp.
/// * A SIGN-FLIPPED moment arm — which the loader's own warning says happens at
///   flexed poses — does not rescale the muscle at all: the QP refuses to
///   recruit it and pins it to `a ≥ aMin` on BOTH sides, where it reads exactly
///   0 % left/right against a true 22.7 %. A finding destroyed and presented as
///   "even".
///
/// # The numerical floor on any of these figures
///
/// OSQP stops on `eps_abs = eps_rel = 1e-3` with polishing off, so an activation
/// carries about 2e-3 of absolute slack and a left/right percentage built from
/// two of them carries a couple of percentage points. `solverNoiseFloor()` is
/// that gap, measured against the proportional rig's known analytic answer, and
/// every claim below is scaled by it rather than by a guess.
final class MomentArmErrorCancellationTests: XCTestCase {

    // MARK: - The rig

    /// A bilateral rig with no shared DOF between the sides — which is what the
    /// real model is: no muscle in `FullBody.osim` spans the left and right
    /// legs. Two DOFs and three muscles per side, one of which spans both DOFs,
    /// so the QP has real redundancy to redistribute through.
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

        static let leftTorques: [String: Double] = ["hip_l": 135, "knee_l": 90]
        /// The proportional case: every right torque this multiple of its left
        /// counterpart. A pure SIZE difference between the legs.
        static let rightScale = 0.8

        /// Right-side torques as a per-joint multiple of the left's, so a case
        /// can differ in PATTERN and not only in size.
        static func torques(hipScale: Double, kneeScale: Double) -> [Double] {
            [leftTorques["hip_l"] ?? 0,
             leftTorques["knee_l"] ?? 0,
             (leftTorques["hip_l"] ?? 0) * hipScale,
             (leftTorques["knee_l"] ?? 0) * kneeScale]
        }

        static var proportionalTorques: [Double] {
            torques(hipScale: rightScale, kneeScale: rightScale)
        }
    }

    /// The rig's analytic answer in the PROPORTIONAL case: with no bound active
    /// the QP is linear in `τ`, so every muscle reads the same figure.
    static var analyticDifferencePercent: Double {
        100 * (1 - Rig.rightScale) / (0.5 * (1 + Rig.rightScale))
    }

    /// How far the unperturbed proportional solve lands from that analytic
    /// answer — i.e. what OSQP's own termination tolerance is worth in these
    /// units.
    private func solverNoiseFloor() throws -> Double {
        let a = try solve(torques: Rig.proportionalTorques)
        var worst = 0.0
        for (base, _, _, _) in Rig.muscles {
            let d = try differencePercent(a, base)
            worst = Swift.max(worst, abs(d - Self.analyticDifferencePercent))
        }
        return worst
    }

    /// One solve on a FRESH solver, so warm starts and `L_MT` history cannot
    /// couple two cases together.
    ///
    /// - Parameter torques: the four joint torques, `hip_l knee_l hip_r knee_r`.
    /// - Parameter scale: per-muscle-base multiplier on BOTH sides' moment arms.
    /// - Parameter leftOnlyScale: per-muscle-base multiplier applied to the LEFT
    ///   side only, on top of `scale`.
    private func solve(torques: [Double],
                       scale: [String: Double] = [:],
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

    /// The finest left/right difference any of the pinned clips can assert,
    /// read from the clips themselves rather than copied — a leak larger than
    /// this is a leak the product cannot tell from a finding.
    private func smallestPublicationFloorOnThePinnedClips() throws -> Double {
        let bundle = Bundle(for: type(of: self))
        var floors: [Double] = []
        for id in GaitClipFixture.allIds {
            let report = try GaitAnalysis.analyse(
                frames: try GaitClipFixture.load(id, bundle: bundle).frames)
            guard report.isUsable else { continue }
            floors.append(report.resolution.resolvableAsymmetryPercent)
        }
        return try XCTUnwrap(floors.min(), "no pinned clip is usable")
    }

    // MARK: - Preconditions, checked rather than assumed

    /// The linearity the whole argument rests on holds only while no activation
    /// sits on a bound. If one did, the QP would be piecewise linear for a
    /// different reason and every number below would mean something else.
    /// Checked on BOTH torque patterns and both perturbations.
    func testNoActivationSitsOnABoundInThisRig() throws {
        let cases: [[String: Double]] = [
            try solve(torques: Rig.proportionalTorques),
            try solve(torques: Rig.proportionalTorques, scale: ["gamma": 0.6]),
            try solve(torques: Rig.torques(hipScale: 0.8, kneeScale: 1.0)),
            try solve(torques: Rig.torques(hipScale: 0.8, kneeScale: 1.0), scale: ["gamma": 0.6]),
        ]
        for activations in cases {
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
        let a = try solve(torques: Rig.proportionalTorques)
        let expected = Self.analyticDifferencePercent
        var measured: [Double] = []
        for (base, _, _, _) in Rig.muscles {
            let d = try differencePercent(a, base)
            measured.append(d)
            // 2 percentage points is the SOLVER's slack, not a tolerance chosen
            // to make this pass: `solverNoiseFloor` is the measured gap and
            // every other assertion in this file is scaled by it.
            XCTAssertEqual(d, expected, accuracy: 2.0,
                           "\(base) should read the rig's \(expected) % left-high")
        }
        print("MOMENT-ARM-METRIC baseline_difference_percent=\(expected) measured=\(measured) "
              + "activations=\(a.sorted { $0.key < $1.key }.map { "\($0.key)=\($0.value)" })")
    }

    // MARK: - The finding

    /// **The retired experiment, kept and labelled.** With the two legs'
    /// torques proportional, a bilateral moment-arm error cannot move the
    /// left/right figure — not because the error cancels in any interesting
    /// sense, but because the QP is linear in `τ` and the answer is
    /// `100(1−c)/(0.5(1+c))` for every muscle whatever `A` is.
    ///
    /// Two things are asserted, and the SECOND is the one that matters:
    ///
    /// 1. the perturbation does not move any figure (the old headline);
    /// 2. **all three muscles read the same figure**, i.e. in the only regime
    ///    where the moment-arm error demonstrably cancels, a per-muscle
    ///    breakdown is one number repeated and carries no per-muscle
    ///    information.
    func testTheProportionalRigIsAnIdentityAndCertifiesNothing() throws {
        let truth = try solve(torques: Rig.proportionalTorques)
        let wrong = try solve(torques: Rig.proportionalTorques, scale: ["gamma": 0.6])
        let noise = try solverNoiseFloor()

        var shift = 0.0
        var figures: [Double] = []
        for (base, _, _, _) in Rig.muscles {
            let dTruth = try differencePercent(truth, base)
            figures.append(dTruth)
            shift = Swift.max(shift, abs(try differencePercent(wrong, base) - dTruth))
        }
        let spread = (figures.max() ?? 0) - (figures.min() ?? 0)
        print("MOMENT-ARM-METRIC proportional_identity worst_shift_pp=\(shift) "
              + "solver_noise_floor_pp=\(noise) figures=\(figures) "
              + "spread_across_muscles_pp=\(spread)")

        XCTAssertLessThanOrEqual(shift, 2 * noise,
                                 "under proportional torques the shift is algebraically zero, so "
                                 + "anything above the solver's slack would mean the QP is not "
                                 + "the one this file documents")
        XCTAssertLessThanOrEqual(spread, 2 * noise,
                                 "and every muscle must read the SAME figure — which is why this "
                                 + "regime cannot support a per-muscle claim: measured spread "
                                 + "\(spread) pp across \(figures.count) muscles")
    }

    /// **THE EXPERIMENT.** Same rig, same bilateral `gamma × 0.6` moment-arm
    /// error, one variable changed: the right leg differs from the left in the
    /// PATTERN of its joint torques (hip 0.80×, knee 1.00×) rather than in their
    /// common size. That is what a gait asymmetry is, and it is the panel's own
    /// worked example — a runner with 200 ms of left contact against 160 ms of
    /// right does not scale both joint moments by one factor.
    ///
    /// The cancellation is then not an identity, and it fails: the error moves a
    /// published left/right figure by more than any pinned clip can resolve.
    /// This test CAN fail — if the QP's redistribution were insensitive to the
    /// moment arms, the shift would sit at the solver's noise floor, and the
    /// per-muscle claim could have stayed.
    func testAShapeAsymmetryMakesABilateralMomentArmErrorLeak() throws {
        let torques = Rig.torques(hipScale: 0.8, kneeScale: 1.0)
        let truth = try solve(torques: torques)
        let wrong = try solve(torques: torques, scale: ["gamma": 0.6])
        let noise = try solverNoiseFloor()
        let floor = try smallestPublicationFloorOnThePinnedClips()

        var worst = 0.0
        var worstBase = ""
        var rows: [String] = []
        for (base, _, _, _) in Rig.muscles {
            let dTruth = try differencePercent(truth, base)
            let dWrong = try differencePercent(wrong, base)
            rows.append(String(format: "%@ %.4f->%.4f (%+.4f pp)", base, dTruth, dWrong,
                               dWrong - dTruth))
            if abs(dWrong - dTruth) > worst {
                worst = abs(dWrong - dTruth)
                worstBase = base
            }
        }
        print("MOMENT-ARM-METRIC shape_asymmetry hip_r=0.80 knee_r=1.00 gamma_scale=0.6 "
              + "worst_left_right_shift_pp=\(worst) on=\(worstBase) "
              + "solver_noise_floor_pp=\(noise) smallest_pinned_clip_floor_percent=\(floor) "
              + "rows=\(rows)")

        XCTAssertGreaterThan(worst, 4 * noise,
                             "the shift must be a real effect and not the solver's slack "
                             + "(shift \(worst) pp, noise \(noise) pp)")
        XCTAssertGreaterThan(worst, floor,
                             "and it must exceed what the best pinned clip can resolve "
                             + "(\(floor) %), or it would be inside the claim floor and the "
                             + "per-muscle comparison could have survived")
    }

    /// **The leak lands on a muscle whose own path is modelled correctly**, so
    /// no per-row "this muscle takes a straight-line path" warning can contain
    /// it. `beta`'s moment arms are untouched by the perturbation; the QP moves
    /// its share of the knee torque anyway, because `gamma` spans the same
    /// joint and now supplies a different amount of it.
    func testTheLeakMovesAMuscleWhosePathIsNotTheOnePerturbed() throws {
        let torques = Rig.torques(hipScale: 0.8, kneeScale: 1.0)
        let truth = try solve(torques: torques)
        let wrong = try solve(torques: torques, scale: ["gamma": 0.6])
        let noise = try solverNoiseFloor()

        let dTruth = try differencePercent(truth, "beta")
        let dWrong = try differencePercent(wrong, "beta")
        print("MOMENT-ARM-METRIC leak_on_unperturbed_muscle beta \(dTruth) -> \(dWrong) "
              + "shift_pp=\(abs(dWrong - dTruth)) noise_pp=\(noise)")

        XCTAssertGreaterThan(abs(dWrong - dTruth), 4 * noise,
                             "beta's own moment arms did not change and its figure moved anyway")
        XCTAssertLessThan(abs(dWrong), abs(dTruth),
                          "and it moved TOWARDS even: a real \(dTruth) % difference is displayed "
                          + "as \(dWrong) %, so the error destroys findings as well as "
                          + "manufacturing them")
    }

    /// **How the leak scales with the asymmetry being measured.** Zero when the
    /// two legs' torques are proportional (`knee_r/knee_l = 0.8`, matching
    /// `hip_r/hip_l`), and growing in both directions away from it — so the
    /// contamination is largest for exactly the runners the product exists to
    /// help, and it cannot be traded away by asking for a better clip.
    func testTheLeakIsZeroAtProportionalTorqueAndGrowsWithTheShapeDifference() throws {
        let noise = try solverNoiseFloor()
        var rows: [String] = []
        var atProportional = Double.infinity
        var worstOverSweep = 0.0
        for kneeScale in [0.6, 0.7, 0.8, 0.9, 0.95, 1.0] {
            let torques = Rig.torques(hipScale: 0.8, kneeScale: kneeScale)
            let truth = try solve(torques: torques)
            let wrong = try solve(torques: torques, scale: ["gamma": 0.6])
            var worst = 0.0
            for (base, _, _, _) in Rig.muscles {
                worst = Swift.max(worst, abs(try differencePercent(wrong, base)
                                             - (try differencePercent(truth, base))))
            }
            rows.append(String(format: "knee_r/knee_l=%.2f worst_shift_pp=%.4f", kneeScale, worst))
            if kneeScale == 0.8 { atProportional = worst }
            worstOverSweep = Swift.max(worstOverSweep, worst)
        }
        print("MOMENT-ARM-METRIC leak_vs_shape_difference hip_r=0.80 gamma_scale=0.6 "
              + "noise_pp=\(noise) \(rows.joined(separator: " "))")

        XCTAssertLessThanOrEqual(atProportional, 2 * noise,
                                 "at knee_r/knee_l = hip_r/hip_l the torques are proportional and "
                                 + "the leak is algebraically zero")
        XCTAssertGreaterThan(worstOverSweep, 10 * atProportional,
                             "and away from that point it is an order of magnitude larger")
    }

    /// **THE SAME EXPERIMENT, WITH THE PERTURBATION RESIZED TO WHAT THIS BUILD
    /// ACTUALLY GETS WRONG.** Gate R3 of `WrappedMomentArmLeakTests`.
    ///
    /// The `×0.6` above is a stand-in for "this muscle's path is a straight line
    /// where the real one wraps around bone". Since 2026-08-08 that is not what
    /// the code does — every `PathWrap` in `FullBody.osim` is solved — so the
    /// honest re-run changes ONE number: the perturbation becomes the residual
    /// disagreement with OpenSim 4.6 that this build still carries, measured
    /// rather than assumed.
    ///
    /// **How that residual is measured, decided before it was read.** The rig's
    /// `gamma` has 30 mm and 50 mm moment arms, so the residual fed to it is
    /// taken over pairs whose REFERENCE arm is at least 20 mm — a relative error
    /// on a 1 mm arm is not a relative error on a 30 mm one, and a muscle with no
    /// leverage at a joint carries no torque there either. It is a p99 and not a
    /// max because the rig has ONE perturbed muscle while the pool has tens of
    /// thousands of pairs, and the max of a large pool is an order statistic; the
    /// max case is computed and printed beside it so nothing is hidden. Both
    /// definitions of the OpenSim reference are pooled, so the worse one counts.
    ///
    /// This is a CROSS-CHECK, not the measurement. The measurement is
    /// `WrappedMomentArmLeakTests`, which perturbs nothing and uses each muscle's
    /// own error on its own arms.
    func testTheShapeAsymmetryLeakWithTheResidualThisBuildLeaves() throws {
        try WrapValidationHarness.build(bundle: Bundle(for: type(of: self)))
        if let failure = WrapValidationHarness.setupFailure { throw XCTSkip(failure) }
        let named = Set(GaitLoadSummary.displayNames.keys)
        var pooled: [Double] = []
        var excluded = 0
        for definitionMatched in [true, false] {
            let measured = WrapValidationHarness.relativeMomentArmResiduals(
                bases: named, minimumReferenceMetres: 0.020,
                definitionMatched: definitionMatched)
            pooled += measured.ratios
            excluded += measured.excludedBelowMinimum
        }
        XCTAssertGreaterThan(pooled.count, 1000,
                             "the residual must be measured over a population, not a handful")
        let p99 = WrapValidationHarness.percentile(pooled, 0.99)
        let maximum = pooled.max() ?? 0
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let noise = try solverNoiseFloor()

        var rows: [String] = []
        var worstAtP99 = 0.0
        var worstAtMax = 0.0
        for (label, residual) in [("p99", p99), ("max", maximum)] {
            var worst = 0.0
            for kneeScale in [0.6, 0.7, 0.8, 0.9, 0.95, 1.0] {
                let torques = Rig.torques(hipScale: 0.8, kneeScale: kneeScale)
                let truth = try solve(torques: torques)
                let wrong = try solve(torques: torques, scale: ["gamma": 1 - residual])
                for (base, _, _, _) in Rig.muscles {
                    worst = Swift.max(worst, abs(try differencePercent(wrong, base)
                                                 - (try differencePercent(truth, base))))
                }
            }
            rows.append(String(format: "%@ residual=%.6f perturbation=x%.6f worst_shift_pp=%.4f",
                               label, residual, 1 - residual, worst))
            if label == "p99" { worstAtP99 = worst } else { worstAtMax = worst }
        }
        print("MOMENT-ARM-METRIC residual_sized_leak pairs=\(pooled.count) "
              + "excluded_below_20mm=\(excluded) "
              + "median_residual=\(WrapValidationHarness.percentile(pooled, 0.5)) "
              + "p99_residual=\(p99) max_residual=\(maximum) \(rows.joined(separator: " ")) "
              + "noise_pp=\(noise) floor_percent=\(floor) "
              + "threshold_pp=\(floor * WrappedMomentArmLeakTests.reopenFractionOfFloor) "
              + "old_measurement_pp=9.92")

        XCTAssertLessThan(worstAtP99,
                          floor * WrappedMomentArmLeakTests.reopenFractionOfFloor,
                          "R3: with the perturbation resized from the guessed x0.6 to the "
                          + "measured residual the leak is \(worstAtP99) pp; the max-residual "
                          + "variant is \(worstAtMax) pp")
    }

    /// **The cross-muscle ordering, still not a measurement — and since
    /// 2026-08-09 the honest statement is about the MARGIN, not about the sort
    /// order.**
    ///
    /// It used to assert that the perturbed list sorts differently. It does not
    /// any more, and that is a real result: with `scaling = 0` and
    /// `polishing = 1` the solver returns the minimiser instead of a point ~14 pp
    /// away from it, and part of what used to reorder this rig was OSQP's own
    /// slack reshuffling two near-tied entries. On the exact answer the order
    /// survives — `[alpha, gamma, beta]` both ways.
    ///
    /// **That is not evidence the ranking is safe, and the replacement says why
    /// in the same units.** `alpha` and `gamma` are separated by 0.0033 of
    /// activation out of 0.60, while a `×0.6` error on ONE muscle's moment arm
    /// moves a ranking key by 0.30 — ninety times the margin that decides which
    /// of them prints first. A sort order that survives that is surviving by
    /// which muscle happened to move, not by a margin. So the assertion is now:
    /// the shift exceeds the smallest gap between adjacent entries. It is
    /// quantitative where the old one was binary, it is in the same units as the
    /// thing it is about, and it does not go green because two numbers landed on
    /// the same side of a comparison.
    ///
    /// None of this reopens the cross-muscle ranking. That is retired for a
    /// STRUCTURAL reason no solver fix can reach — nothing puts two different
    /// muscles' efforts on one scale, because the sharing step divides by each
    /// muscle's own leverage and its own maximum force — and
    /// `MuscleOverlay.update(joints:)` takes no muscle solve at all.
    func testABilateralMomentArmErrorMovesTheCrossMuscleQuantityPastItsRankingMargins() throws {
        let truth = try solve(torques: Rig.proportionalTorques)
        let wrong = try solve(torques: Rig.proportionalTorques, scale: ["gamma": 0.6])
        var worstRankingShiftPercent = 0.0
        var worstRankingShiftAbsolute = 0.0
        var truthKeys: [String: Double] = [:]
        var wrongKeys: [String: Double] = [:]
        for (base, _, _, _) in Rig.muscles {
            let t = try rankingKey(truth, base)
            let w = try rankingKey(wrong, base)
            truthKeys[base] = t
            wrongKeys[base] = w
            worstRankingShiftPercent = Swift.max(worstRankingShiftPercent, 100 * abs(w - t) / t)
            worstRankingShiftAbsolute = Swift.max(worstRankingShiftAbsolute, abs(w - t))
        }
        let bases = Rig.muscles.map(\.base)
        let truthOrder = bases.sorted { (truthKeys[$0] ?? 0) > (truthKeys[$1] ?? 0) }
        let wrongOrder = bases.sorted { (wrongKeys[$0] ?? 0) > (wrongKeys[$1] ?? 0) }
        // The margin the sort order actually rests on: the smallest distance
        // between two adjacent entries in the TRUE ranking.
        let sortedKeys = truthOrder.compactMap { truthKeys[$0] }
        let gaps = zip(sortedKeys, sortedKeys.dropFirst()).map { $0 - $1 }
        let smallestGap = gaps.min() ?? .infinity
        print("MOMENT-ARM-METRIC cross_muscle worst_ranking_shift_percent="
              + "\(worstRankingShiftPercent) worst_ranking_shift_absolute="
              + "\(worstRankingShiftAbsolute) smallest_adjacent_gap=\(smallestGap) "
              + "shift_over_gap=\(worstRankingShiftAbsolute / smallestGap) "
              + "keys_truth=\(truthOrder.map { ($0, truthKeys[$0] ?? 0) }) "
              + "order_truth=\(truthOrder) order_wrong=\(wrongOrder) "
              + "order_changed=\(truthOrder != wrongOrder)")

        XCTAssertGreaterThan(worstRankingShiftPercent, 10.0)
        XCTAssertGreaterThan(worstRankingShiftAbsolute, smallestGap,
                             "one muscle's moment-arm error moves a ranking key by "
                             + "\(worstRankingShiftAbsolute) while the closest pair in the true "
                             + "ranking is separated by \(smallestGap) — if that ever reverses, "
                             + "the ranking has a margin and this retirement's evidence has to be "
                             + "re-read")
    }

    /// **The sign flip the loader's own warning names, and what it actually
    /// does.**
    ///
    /// A muscle whose modelled moment arm has the wrong sign is not merely
    /// rescaled — the QP refuses to recruit it and pushes it onto the
    /// `a ≥ aMin` bound on BOTH sides, where it reads exactly 0 % left/right.
    /// That is a real finding turned into "even".
    func testASignFlippedMomentArmPinsTheMuscleToTheFloorAndReadsFalselyEven() throws {
        let truth = try solve(torques: Rig.proportionalTorques)
        let flipped = try solve(torques: Rig.proportionalTorques, scale: ["gamma": -1.0])
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
        XCTAssertTrue(GaitLoadSummary.activationFloorThreshold > MuscleSolver().minActivation)
    }

    /// **A one-sided error**, i.e. the two legs modelled differently, or the
    /// same wrap error evaluated at two poses far enough apart to differ. Each
    /// side IS sampled at its own mid-contact, and nothing checks that those
    /// poses are comparable, so this is not hypothetical.
    func testAOneSidedMomentArmErrorDoesNotCancel() throws {
        let truth = try solve(torques: Rig.proportionalTorques)
        let oneSided = try solve(torques: Rig.proportionalTorques,
                                 leftOnlyScale: ["gamma": 0.6])
        let dOneSided = try differencePercent(oneSided, "gamma")
        let dTruth = try differencePercent(truth, "gamma")
        let shift = abs(dOneSided - dTruth)
        print("MOMENT-ARM-METRIC one_sided_perturbation gamma_scale_left_only=0.6 "
              + "left_right_shift_pp=\(shift)")
        XCTAssertGreaterThan(shift, 10.0,
                             "a one-sided error must move the left/right figure — this was the "
                             + "assumption the retired claim rested on, and it is worth this much")
    }

    /// **The consequence, pinned where a future reader cannot miss it.** No
    /// clip, however clean, may state a per-muscle left/right difference in this
    /// build; the retirement is a property of the model, not of the data.
    func testNoClipMayStateAPerMuscleLeftRightDifference() throws {
        XCTAssertFalse(GaitLoadSummary.perMuscleLeftRightClaimIsSupported,
                       "the leak above is larger than the pinned clips' floors and nothing in "
                       + "this pipeline bounds it")
        let summary = GaitLoadSummaryTests.summary(resolvable: 5)
        let huge = GaitLoadSummaryTests.load(differencePercent: 200)
        XCTAssertTrue(summary.clearsStatisticalFloor(huge),
                      "a 200 % difference clears every statistical floor there is")
        XCTAssertFalse(summary.permits(huge),
                       "and is still not permitted, because the statistics were never the "
                       + "binding constraint")
    }
}
