import XCTest
@testable import BioMotion

/// **Does the moment-arm error that survives path wrapping still leak into a
/// LEFT/RIGHT comparison?** This is the re-measurement the retirement of the
/// per-muscle claim registered as its own falsifier, run on REAL geometry
/// instead of a three-muscle rig.
///
/// # What the retirement rested on, and what has changed under it
///
/// `MomentArmErrorCancellationTests` retired the claim on two halves. The first
/// — the QP is linear in `τ`, so a proportional right leg makes any moment-arm
/// perturbation cancel by identity — is algebra and has not changed. The second
/// is a MEASUREMENT: give the right leg a different torque SHAPE and a
/// bilateral `×0.6` moment-arm error moves a published figure by **9.92 pp**,
/// against publication floors of 8.086 % / 10.145 %.
///
/// `×0.6` was a stand-in for "this muscle's path is a straight line where the
/// real one wraps around bone", and on 2026-08-08 that stopped being what the
/// code does: cylinder and then ellipsoid wrapping shipped, all 76 `PathWrap`
/// references in `FullBody.osim` are solved, and the residual against OpenSim
/// 4.6 is millimetres rather than the 146.6 mm the straight line was out by.
/// The 9.92 pp is therefore a measurement of a defect that no longer exists. It
/// is NOT evidence about the defect that does.
///
/// # The rig, and why it is not the three-muscle one
///
/// `MomentArmErrorCancellationTests` perturbs one synthetic muscle by a number
/// somebody chose. This file perturbs nothing: it takes the moment arms this
/// build actually computes, and the moment arms OpenSim computes for the same
/// model at the same pose, and solves the SAME QP twice.
///
/// * **Geometry** — the 40 right-leg muscles of `FullBody.osim` that span at
///   least one of the six unlocked right-leg coordinates, at every non-arm pose
///   the shared `WrapValidationHarness` sweep visits.
/// * **The bilateral structure** — the left leg is the right leg's exact
///   MIRROR: same muscles, same moment arms, acting on the six left
///   coordinates. That is what makes the modelling error bilateral BY
///   CONSTRUCTION, which is the case the cancellation argument was about. A
///   real stride samples the two legs at two poses; that is a different and
///   larger effect (the one-sided bound, 23.8 pp with the old arms) and this
///   file does not measure it.
/// * **The asymmetry** — the left leg's joint torques are the right's with a
///   per-joint scale, so the two legs differ in the SHAPE of their torque and
///   not in its size. Joint torques come from inverse dynamics, which never
///   touches a moment arm, so `τ` is IDENTICAL in the two solves. The only
///   thing that differs is `A`.
/// * **The statistic** — `100·(a_l − a_r)/mean`, the panel's own.
/// * **The force scale** — every muscle is held at its own `l_opt + l_Ts` with
///   zero pennation and zero velocity, so `f_AL = f_FV = cos α = 1` and the QP's
///   `A` is exactly `R · diag(F_max)`. That is the same convention the
///   three-muscle rig uses, and it is what makes the objective reproducible
///   outside `MuscleSolver` — see the next section.
///   `testTheForceScaleIsExactlyFmaxAtTheseLengths` checks it against the
///   solver's own returned forces rather than assuming it.
///
/// # THE INSTRUMENT: the shipping solver is not fine enough to measure this
///
/// The first run of this file took its maximum over every cell and reported a
/// 54 pp leak on a cell with 9 readable muscles. That number was not a
/// moment-arm effect. `MuscleSolver` runs OSQP at `eps_abs = eps_rel = 1e-3`
/// with polishing off and accepts `OSQP_SOLVED_INACCURATE`, so an activation
/// carries up to 0.02 of ABSOLUTE slack; a left/right percentage built from two
/// of them carries `≈ 100·2·0.02/ā`, tens of percentage points at a realistic
/// `ā`. The three-muscle rig does not see this — its QP is small enough that
/// OSQP solves it accurately — and neither did anything else in this project.
///
/// So this file solves the SAME objective a second time, to machine precision,
/// with `BoxQP` (an active-set solver using Woodbury on the twelve coordinate
/// rows), and reports THREE quantities rather than one:
///
/// * `leakExact` = `|d(ours, exact) − d(truth, exact)|` — the moment arms alone.
/// * `solverSlack` = `|d(ours, OSQP) − d(ours, exact)|` — the shipping solver
///   alone, at fixed geometry.
/// * `leakShipped` = `|d(ours, OSQP) − d(truth, exact)|` — what the product's
///   published number would actually be out by, both causes together.
///
/// # Pre-registered gates
///
/// `floor` is read from the pinned clips, not copied: the smallest
/// `resolvableAsymmetryPercent` any usable pinned clip achieves (8.086 % at the
/// time of writing). Each quantity is maximised over the screened bases, the
/// pose set, the torque-shape sweep, the effort sweep and BOTH definitions of
/// the OpenSim reference.
///
/// **REOPEN** — flip `GaitLoadSummary.perMuscleLeftRightClaimIsSupported` to
/// `true` — requires ALL of:
///
/// * **R1** `leakExact < floor / 5`. Why a fifth and not "below the floor": the
///   floor is a 95 % half-width on the statistic's RANDOM error, and the leak is
///   a BIAS that adds to it. In a normal approximation a bias of `h/5` on a
///   half-width `h = 1.96σ` moves a nominal 5 % false-positive rate to 6.8 %;
///   `h/3` moves it to 10.0 %, i.e. doubles it. A fifth is the largest bias that
///   leaves the quoted confidence roughly what it says.
/// * **R2** `leakShipped < floor / 5` — the same bar applied to the number the
///   product would actually print, so a clean moment arm cannot be cancelled out
///   by a sloppy solve. (Amended: see L-A4.)
/// * **R3** the three-muscle rig re-run with the perturbation resized from the
///   guessed `×0.6` to the MEASURED residual also lands under `floor / 5`
///   (`MomentArmErrorCancellationTests.testTheShapeAsymmetryLeakWithTheResidualThisBuildLeaves`).
/// * **R4** `unmodelledPathWraps == 0` on both shipped models, and no muscle in
///   `GaitLoadSummary.displayNames` has an unmodelled path.
/// * **R5 — THE CONTROL.** The identical rig, driven with the STRAIGHT-LINE
///   moment arms this project shipped until 2026-08-08, leaks MORE than the
///   floor. Without it a small leak is indistinguishable from a rig too blunt to
///   detect anything.
/// * **R6** at least 20 screened bases at the worst-case cell, so the maximum is
///   taken over a population rather than over a handful, and at least 30 cells.
/// * **R7 — the claim must be INFORMATIVE as well as safe.** The retirement's
///   second half was that the regime where the error cancels is the regime where
///   every muscle reads the same number, so there is no regime that is both.
///   That is an argument about a RATIO: require the MEDIAN over the cells of
///   (spread of the true left/right figures across muscles) / (leakShipped) to
///   exceed 4. The minimum is reported too — a cell whose ratio is low is a
///   near-proportional configuration, where the rows carry little per-muscle
///   information for a reason that predates this work and is unchanged by it.
///
/// **STAYS RETIRED** if any of: a gated quantity `≥ floor` (W1); a gated
/// quantity in `[floor/5, floor)` — "comparable to the floor" is not "below" it
/// (W2); R4 fails (W3); R5 fails, because then the rig proves nothing either way
/// (W4); R7 fails (W5).
///
/// No threshold here may be adjusted after a number is read. If the measurement
/// lands between the bands, the claim stays retired and the number is reported.
///
/// # Amendments, with the mechanism that forced each
///
/// * **L-A1/L-A2, WITHDRAWN.** The first two amendments tried to rescue the
///   original instrument by discarding cells whose "solver noise floor" was
///   large and by widening the effort ladder. Both were built on a
///   mis-specified instrument: that noise floor solved the same arms under a
///   PROPORTIONAL torque and compared against the analytic `100(c−1)/(0.5(1+c))`,
///   which is the right answer only while NO activation sits on a bound. In an
///   80-muscle rig most muscles sit on `aMin`, and a clamped muscle's
///   contribution does not scale with `τ`, so the interior muscles compensate
///   differently at different torque scales. The 18–45 pp it reported was the
///   ACTIVE SET, not the solver — a genuine nonlinearity of the QP, and not
///   evidence about anything this file measures. Both amendments are withdrawn
///   and replaced by L-A4. The effort ladder stays widened, which costs nothing.
/// * **L-A3 — a sign error in that instrument, found on the way.** It compared
///   against `+100(1−c)/(0.5(1+c))` while this rig scales the LEFT leg, so the
///   analytic answer is negative. It reported ~44 pp of pure sign.
/// * **L-A4 — the solver's contribution is now measured, not estimated.** R2 was
///   registered as `leak + 2·noise < floor` with `noise` from the instrument
///   above. With that instrument withdrawn, R2 becomes a DIRECT measurement
///   against a machine-precision solve of the same objective, at the same
///   `floor/5` bar as R1. That is strictly stronger than what was registered:
///   the original admitted a leak up to `floor` and this one does not.
///
/// # WHAT IT MEASURED — 2026-08-09, 582 readable cells
///
/// **The claim stays retired, and the binding constraint has changed.**
///
/// * **The wrap solver did what it was for.** Median moment-arm leak
///   **0.977 pp** against the straight line's **7.939 pp** on the identical rig
///   — 8.1× — and the three-muscle rig re-run with the perturbation resized from
///   the guessed `×0.6` to the measured p99 residual (1.114 %) reads **0.568 pp**
///   where it read **9.92 pp**. R3 passes.
/// * **R1 fails on the tail.** Worst moment-arm leak **123.10 pp** (`piri` at
///   `grid_h060_k000_a+00`, against the central-difference reference); against
///   the analytic reference alone, max 42.46 / p99 9.94 / median 0.41. OpenSim's
///   two columns disagree with each other by more than the gates allow, so which
///   part of that tail is this build's residual and which is the reference's own
///   inconsistency is NOT settled here.
/// * **R2 fails on something that is not the moment arms at all.** With the
///   geometry held fixed, OSQP's answer differs from the exact minimiser of the
///   SAME objective by a median of **14.88 pp**, p90 37.83, max **100.98 pp**, at
///   a median activation of 0.132 — against the 0.02 absolute tolerance this
///   solver accepts, which predicts `100·2·0.02/0.132 = 30 pp`. The mechanism
///   is arithmetic, not a bug: an absolute stopping tolerance on `a` becomes a
///   relative error in `100·(a_l − a_r)/mean`.
/// * **R7 fails as a consequence of R2, and only of R2**: median
///   spread-over-error **2.90** against the required 4 when the error is the
///   number the product would print, and **48.52** when it is the moment-arm
///   error alone. So the retirement's second half — "the regime where the error
///   cancels is the regime where every muscle reads the same number, and there
///   is no regime that is both safe and informative" — is DEFEATED as an
///   argument about moment arms: the rows carry 48× more per-muscle signal than
///   moment-arm error. They are drowned by the solver instead.
/// * **R4 and R5 and R6 pass**: 0 unmodelled `PathWrap`s on both models, the
///   straight-line control leaks 66.88 pp (271 of 549 cells over the floor), 582
///   readable cells.
///
/// So the honest statement is no longer "the moment arms are too wrong to
/// publish a per-muscle number". It is "the moment arms are now the SMALLER of
/// two errors, and the larger one is the solver tolerance the product ships".
final class WrappedMomentArmLeakTests: XCTestCase {

    // MARK: - Pre-registered constants

    /// R1, R2. A fifth of the publication floor.
    static let reopenFractionOfFloor = 0.2
    /// R6.
    static let minimumScreenedBases = 20
    static let minimumCells = 30
    /// R7.
    static let minimumInformationToLeakRatio = 4.0
    /// How far inside the box a muscle has to be, in the EXACT solution, to be
    /// read. A muscle within a thousandth of `aMin` is one OSQP cannot tell from
    /// the bound, and the panel excludes bound muscles for the same reason.
    static let interiorMargin = 1e-3

    // MARK: - The rig

    /// The six unlocked right-leg coordinates. `mtp_angle_r` is excluded because
    /// `FullBody.osim` locks it and the reference therefore carries no value for
    /// it — a locked coordinate's `computeMomentArm` is a refusal (exactly 0.0),
    /// not a measurement.
    static let rightLegCoordinates = ["hip_flexion_r", "hip_adduction_r", "hip_rotation_r",
                                      "knee_angle_r", "ankle_angle_r", "subtalar_angle_r"]
    static let leftLegCoordinates = ["hip_flexion_l", "hip_adduction_l", "hip_rotation_l",
                                     "knee_angle_l", "ankle_angle_l", "subtalar_angle_l"]
    static let dofNames = rightLegCoordinates + leftLegCoordinates

    /// Where a moment arm comes from. The two `reference` cases are OpenSim's two
    /// mutually-inconsistent answers for the same quantity — see the "readings
    /// that lie" entry on `computeMomentArm` — and the gates are taken over the
    /// WORSE of them rather than over the flattering one.
    enum ArmSource: String, CaseIterable {
        /// This build: `−dL/dq` with every `PathWrap` solved.
        case ours
        /// What shipped until 2026-08-08: OpenSim with every `WrapObject`
        /// deactivated, which the 2026-08-08 measurement showed this code
        /// reproduced to 4.39 mm. The CONTROL.
        case straightLine
        /// OpenSim's `GeometryPath::computeMomentArm`, wraps solved.
        case analytic
        /// OpenSim's own central difference of its own length — the column that
        /// is definition-matched to a `−dL/dq` implementation. Falls back to
        /// `analytic` for rows the finite-difference fixture does not carry,
        /// which is exactly the muscles with no `PathWrap`, where the two
        /// columns are the same number.
        case centralDifference

        var isReference: Bool { self == .analytic || self == .centralDifference }
    }

    static func value(_ sample: WrapValidationHarness.Sample, _ source: ArmSource) -> Double {
        switch source {
        case .ours: return sample.ours
        case .straightLine: return sample.wrapOff
        case .analytic: return sample.wrapOn
        case .centralDifference: return sample.centralDifference ?? sample.wrapOn
        }
    }

    // MARK: - Index over the shared sweep, built once

    /// `muscle name → coordinate → sample`, for one pose.
    typealias PoseTable = [String: [String: WrapValidationHarness.Sample]]

    private(set) static var byPose: [String: PoseTable] = [:]
    private(set) static var poseOrder: [String] = []
    private(set) static var bases: [String] = []
    private static var built = false

    /// Arm-sweep poses hold the LEGS fixed, so including them would multiply the
    /// cell count by near-copies of one leg configuration and make a maximum over
    /// cells look better sampled than it is.
    static let excludedPosePrefixes = ["elbow_sweep_", "shoulder_sweep_"]

    /// Membership is decided by the FIXTURE's structural span, never by a
    /// moment-arm magnitude, so which muscles are in the rig cannot depend on
    /// which arm source is under test.
    static func buildIndex() {
        guard !built else { return }
        built = true
        var tables: [String: PoseTable] = [:]
        let legs = Set(rightLegCoordinates)
        for sample in WrapValidationHarness.samples {
            guard legs.contains(sample.coordinate), sample.muscle.hasSuffix("_r") else { continue }
            guard !excludedPosePrefixes.contains(where: { sample.pose.hasPrefix($0) }) else {
                continue
            }
            tables[sample.pose, default: [:]][sample.muscle, default: [:]][sample.coordinate] = sample
        }
        byPose = tables
        poseOrder = tables.keys.sorted()
        var found = Set<String>()
        for table in tables.values {
            for muscle in table.keys where WrapValidationHarness.muscleParameters[muscle] != nil {
                if let split = GaitLoadSummary.split(muscle) { found.insert(split.base) }
            }
        }
        bases = found.sorted()
    }

    /// The matrices the two solvers are handed. Separate from the solve so a test
    /// can inspect the mirror rather than trust a comment about it.
    struct Built {
        let names: [String]
        /// Row-major `[names.count × 12]`, moment arms in metres.
        let arms: [Double]
        /// The same, scaled by `F_max` — the QP's own `A`, transposed.
        let armsInForceUnits: [Double]
        let lengths: [Double]
        let maxForces: [Double]
        let optimalFiberLengths: [Double]
        let tendonSlackLengths: [Double]
    }

    static func build(pose: String, source: ArmSource) -> Built? {
        guard let table = byPose[pose] else { return nil }
        var names: [String] = []
        var arms: [Double] = []
        var scaled: [Double] = []
        var lengths: [Double] = []
        var maxForces: [Double] = []
        var optimal: [Double] = []
        var slack: [Double] = []
        let columns = dofNames.count
        for base in bases {
            let rightName = "\(base)_r"
            guard let row = table[rightName],
                  let parameters = WrapValidationHarness.muscleParameters[rightName],
                  parameters.maxForce > 0, parameters.optimalFiberLength > 0 else { continue }
            var right = [Double](repeating: 0, count: columns)
            for (slot, coordinate) in rightLegCoordinates.enumerated() {
                if let sample = row[coordinate] { right[slot] = value(sample, source) }
            }
            // The left leg IS the right leg, mirrored: identical arms, moved to
            // the left coordinates. That makes the modelling error bilateral by
            // construction rather than by assumption.
            var left = [Double](repeating: 0, count: columns)
            for slot in 0..<rightLegCoordinates.count { left[slot + 6] = right[slot] }
            // Isometric: normalised fibre length 1, so f_AL = f_FV = cos α = 1.
            let isometric = parameters.optimalFiberLength + parameters.tendonSlackLength
            for (name, arm) in [(rightName, right), ("\(base)_l", left)] {
                names.append(name)
                arms.append(contentsOf: arm)
                scaled.append(contentsOf: arm.map { $0 * parameters.maxForce })
                lengths.append(isometric)
                maxForces.append(parameters.maxForce)
                optimal.append(parameters.optimalFiberLength)
                slack.append(parameters.tendonSlackLength)
            }
        }
        guard !names.isEmpty else { return nil }
        return Built(names: names, arms: arms, armsInForceUnits: scaled, lengths: lengths,
                     maxForces: maxForces, optimalFiberLengths: optimal,
                     tendonSlackLengths: slack)
    }

    struct SolveOutcome {
        /// What `MuscleSolver` returns — OSQP at its shipping tolerance.
        let shipped: [String: Double]
        /// The same objective at machine precision.
        let exact: [String: Double]
        let kktResidual: Double
        let converged: Bool
        let relativeTorqueResidual: Double
        let atLower: Int
        let atUpper: Int
    }

    /// One QP on a FRESH `MuscleSolver` — no warm start, no `L_MT` history — plus
    /// the same objective solved exactly.
    static func solve(pose: String, source: ArmSource, torques: [Double]) -> SolveOutcome? {
        guard let model = build(pose: pose, source: source) else { return nil }
        guard let result = MuscleSolver().solveReal(
            withJointTorques: torques.map(NSNumber.init(value:)),
            momentArms: model.arms.map(NSNumber.init(value:)),
            muscleNames: model.names,
            muscleLengths: model.lengths.map(NSNumber.init(value:)),
            maxForces: model.maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: model.optimalFiberLengths.map(NSNumber.init(value:)),
            tendonSlackLengths: model.tendonSlackLengths.map(NSNumber.init(value:)),
            pennationAngles: model.names.map { _ in NSNumber(value: 0.0) },
            jointVelocities: dofNames.map { _ in NSNumber(value: 0.0) },
            dofNames: dofNames,
            dt: 1.0 / 30.0,
            softPenalty: 100.0) else { return nil }
        var shipped: [String: Double] = [:]
        for (index, name) in result.muscleNames.enumerated() {
            shipped[name] = result.activations[index].doubleValue
        }
        let precise = BoxQP.solve(arms: model.armsInForceUnits, nMuscles: model.names.count,
                                  nDOFs: dofNames.count, torques: torques,
                                  lower: MuscleSolver().minActivation,
                                  upper: MuscleSolver.maxActivation)
        var exact: [String: Double] = [:]
        for (index, name) in model.names.enumerated() { exact[name] = precise.activations[index] }
        return SolveOutcome(shipped: shipped, exact: exact, kktResidual: precise.kktResidual,
                            converged: result.converged,
                            relativeTorqueResidual: result.relativeTorqueResidual,
                            atLower: precise.activeAtLower, atUpper: precise.activeAtUpper)
    }

    /// The joint torques a leg at this pose would need to hold every one of its
    /// muscles at `activation`, computed through the REFERENCE moment arms.
    /// Inverse dynamics never touches a moment arm, so the same `τ` is handed to
    /// both solves; this construction only has to be a plausible, feasible one.
    static func torques(pose: String, reference: ArmSource, activation: Double,
                        shape: [Double]) -> [Double]? {
        guard let table = byPose[pose] else { return nil }
        var right = [Double](repeating: 0, count: rightLegCoordinates.count)
        for base in bases {
            let name = "\(base)_r"
            guard let row = table[name],
                  let parameters = WrapValidationHarness.muscleParameters[name] else { continue }
            for (slot, coordinate) in rightLegCoordinates.enumerated() {
                guard let sample = row[coordinate] else { continue }
                right[slot] += value(sample, reference) * activation * parameters.maxForce
            }
        }
        return right + zip(right, shape).map { $0 * $1 }
    }

    /// `100·(L − R)/mean` — the panel's own statistic.
    static func differencePercent(_ activations: [String: Double], _ base: String) -> Double? {
        guard let l = activations["\(base)_l"], let r = activations["\(base)_r"] else { return nil }
        let mean = 0.5 * (l + r)
        guard mean > 0 else { return nil }
        return 100 * (l - r) / mean
    }

    /// A base is READABLE when all four of its EXACT activations sit strictly
    /// inside the box. Screening on the exact solution rather than on OSQP's is
    /// the point: the true active set is a property of the problem, and letting
    /// the solver decide which muscles are readable would make the screen depend
    /// on the thing being measured.
    static func isScreened(_ truth: SolveOutcome, _ test: SolveOutcome, _ base: String) -> Bool {
        let lower = MuscleSolver().minActivation + interiorMargin
        let upper = MuscleSolver.maxActivation - interiorMargin
        for outcome in [truth, test] {
            for side in ["l", "r"] {
                guard let value = outcome.exact["\(base)_\(side)"] else { return false }
                if value <= lower || value >= upper { return false }
            }
        }
        return true
    }

    // MARK: - The sweep

    struct Cell {
        let pose: String
        let reference: ArmSource
        let subject: ArmSource
        let shape: String
        let effort: Double
        let screened: Int
        /// `|d(subject, exact) − d(reference, exact)|` — the moment arms alone.
        let leakExact: Double
        /// `|d(subject, OSQP) − d(subject, exact)|` — the shipping solver alone.
        let solverSlack: Double
        /// `|d(subject, OSQP) − d(reference, exact)|` — both causes together.
        let leakShipped: Double
        let worstBase: String
        /// `leakShipped` restricted to muscles `displayNames` prints a row for.
        let leakAmongNamed: Double
        let namedScreened: Int
        /// Spread of the TRUE left/right figures across the screened bases.
        let trueSpread: Double
        let converged: Bool
        let kktResidual: Double
        let medianActivation: Double
        let torqueResidual: Double
    }

    /// The torque shapes. `hip 0.80 / knee 1.00` is the case that retired the
    /// claim; the sweep around it exists because the leak is zero at the
    /// proportional point and grows away from it, so one point is not a
    /// measurement of the worst case.
    static let shapes: [(name: String, scales: [Double])] = [
        ("hip0.80_knee0.60", [0.8, 0.8, 0.8, 0.6, 1.0, 1.0]),
        ("hip0.80_knee0.80", [0.8, 0.8, 0.8, 0.8, 1.0, 1.0]),
        ("hip0.80_knee1.00", [0.8, 0.8, 0.8, 1.0, 1.0, 1.0]),
        ("hip1.00_knee0.80", [1.0, 1.0, 1.0, 0.8, 1.0, 1.0]),
        ("hip0.90_ankle1.20", [0.9, 0.9, 0.9, 1.0, 1.2, 1.2]),
    ]

    /// How hard the leg is working. Swept because it decides WHICH muscles sit on
    /// a bound and how much of the solver's ABSOLUTE slack reaches a RELATIVE
    /// statistic — not what the exact leak is. Where no bound is active the QP is
    /// linear in `τ`, so a common scale divides out of `100·(a_l − a_r)/mean`.
    static let effortLevels = [0.3, 0.9, 2.7]

    private static var cells: [Cell] = []
    private static var cellsBuilt = false

    static func sweep() -> [Cell] {
        guard !cellsBuilt else { return cells }
        buildIndex()
        let named = Set(bases.filter { GaitLoadSummary.displayNames[$0] != nil })
        var out: [Cell] = []
        for pose in poseOrder {
            for reference in ArmSource.allCases where reference.isReference {
                for effort in effortLevels {
                    for shape in shapes {
                        guard let tau = torques(pose: pose, reference: reference,
                                                activation: effort, shape: shape.scales),
                              tau.contains(where: { abs($0) > 1e-9 }),
                              let truth = solve(pose: pose, source: reference, torques: tau)
                        else { continue }
                        for subject in ArmSource.allCases where !subject.isReference {
                            guard let test = solve(pose: pose, source: subject, torques: tau)
                            else { continue }
                            var exactLeak = 0.0, slack = 0.0, shippedLeak = 0.0, namedLeak = 0.0
                            var worstBase = "", screened = 0, namedScreened = 0
                            var trueFigures: [Double] = []
                            var levels: [Double] = []
                            for base in bases {
                                guard isScreened(truth, test, base),
                                      let dTruth = differencePercent(truth.exact, base),
                                      let dExact = differencePercent(test.exact, base),
                                      let dShipped = differencePercent(test.shipped, base)
                                else { continue }
                                screened += 1
                                trueFigures.append(dTruth)
                                if let l = truth.exact["\(base)_l"],
                                   let r = truth.exact["\(base)_r"] {
                                    levels.append(0.5 * (l + r))
                                }
                                exactLeak = Swift.max(exactLeak, abs(dExact - dTruth))
                                slack = Swift.max(slack, abs(dShipped - dExact))
                                let total = abs(dShipped - dTruth)
                                if total > shippedLeak { shippedLeak = total; worstBase = base }
                                if named.contains(base) {
                                    namedScreened += 1
                                    namedLeak = Swift.max(namedLeak, total)
                                }
                            }
                            out.append(Cell(pose: pose, reference: reference, subject: subject,
                                            shape: shape.name, effort: effort, screened: screened,
                                            leakExact: exactLeak, solverSlack: slack,
                                            leakShipped: shippedLeak, worstBase: worstBase,
                                            leakAmongNamed: namedLeak, namedScreened: namedScreened,
                                            trueSpread: (trueFigures.max() ?? 0)
                                                      - (trueFigures.min() ?? 0),
                                            converged: truth.converged && test.converged,
                                            kktResidual: Swift.max(truth.kktResidual,
                                                                   test.kktResidual),
                                            medianActivation: WrapValidationHarness.percentile(
                                                levels, 0.5),
                                            torqueResidual: Swift.max(truth.relativeTorqueResidual,
                                                                      test.relativeTorqueResidual)))
                        }
                    }
                }
            }
        }
        cells = out
        cellsBuilt = true
        return out
    }

    /// R6: a cell is read only where the exact solver reached a KKT point and at
    /// least `minimumScreenedBases` muscles are interior.
    static func readable(_ subject: ArmSource, in cells: [Cell]) -> [Cell] {
        cells.filter {
            $0.subject == subject && $0.screened >= minimumScreenedBases
                && $0.kktResidual < 1e-6 && $0.converged
        }
    }

    // MARK: - The floor

    /// The finest left/right difference any pinned clip can assert, read from the
    /// clips rather than copied — same helper as
    /// `MomentArmErrorCancellationTests`.
    func smallestPublicationFloorOnThePinnedClips() throws -> Double {
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

    // MARK: - Structure, checked before anything is concluded from it

    override func setUpWithError() throws {
        try WrapValidationHarness.build(bundle: Bundle(for: type(of: self)))
        if let failure = WrapValidationHarness.setupFailure { throw XCTSkip(failure) }
        Self.buildIndex()
    }

    /// The rig is what it says: real muscles, real coordinates, and a muscle
    /// population that does not depend on which arm source is under test.
    func testTheRigIsBuiltFromTheModelAndNotFromAList() throws {
        let named = Set(Self.bases).intersection(GaitLoadSummary.displayNames.keys)
        print("LEAK-METRIC rig bases=\(Self.bases.count) poses=\(Self.poseOrder.count) "
              + "dofs=\(Self.dofNames.count) named=\(named.count) "
              + "named_list=\(named.sorted()) poses_list=\(Self.poseOrder)")
        XCTAssertGreaterThanOrEqual(Self.bases.count, 40)
        XCTAssertGreaterThanOrEqual(Self.poseOrder.count, 20)
        for pose in Self.poseOrder {
            let table = try XCTUnwrap(Self.byPose[pose])
            for muscle in table.keys { XCTAssertTrue(muscle.hasSuffix("_r")) }
        }
        for base in Self.bases {
            XCTAssertNotNil(WrapValidationHarness.muscleParameters["\(base)_r"])
        }
        XCTAssertGreaterThanOrEqual(named.count, 25,
                                    "the rig has to contain the muscles the product would name")
    }

    /// **The mirror is exact in the matrix the solver is handed**, so the
    /// modelling error really is bilateral. Read off `Built.arms`, not asserted
    /// in a comment: a left row must be zero on every right coordinate, equal to
    /// its right twin on every left one, and the pair must not be all-zero.
    func testTheLeftLegIsTheRightLegsExactMirrorInTheMatrixTheSolverSees() throws {
        var checked = 0
        for pose in [Self.poseOrder.first, Self.poseOrder.last].compactMap({ $0 }) {
            for source in Self.ArmSource.allCases {
                let model = try XCTUnwrap(Self.build(pose: pose, source: source))
                let columns = Self.dofNames.count
                XCTAssertEqual(model.arms.count, model.names.count * columns)
                XCTAssertEqual(model.names.count % 2, 0)
                for pair in stride(from: 0, to: model.names.count, by: 2) {
                    XCTAssertTrue(model.names[pair].hasSuffix("_r"))
                    XCTAssertTrue(model.names[pair + 1].hasSuffix("_l"))
                    var anyNonZero = false
                    for slot in 0..<6 {
                        let right = model.arms[pair * columns + slot]
                        XCTAssertEqual(model.arms[(pair + 1) * columns + slot + 6], right,
                                       accuracy: 0,
                                       "\(model.names[pair]) is not mirrored at slot \(slot)")
                        XCTAssertEqual(model.arms[pair * columns + slot + 6], 0)
                        XCTAssertEqual(model.arms[(pair + 1) * columns + slot], 0)
                        if right != 0 { anyNonZero = true }
                        checked += 1
                    }
                    XCTAssertTrue(anyNonZero, "\(model.names[pair]) has an all-zero row")
                    XCTAssertEqual(model.lengths[pair], model.lengths[pair + 1])
                    XCTAssertEqual(model.maxForces[pair], model.maxForces[pair + 1])
                }
            }
        }
        XCTAssertGreaterThan(checked, 500)
    }

    /// **`A = R · diag(F_max)` is asserted against the solver's own output, not
    /// assumed.** `BoxQP` reproduces `MuscleSolver`'s objective only if the Hill
    /// multipliers are all 1 at these lengths; if a future change to the force
    /// model breaks that, every exact number in this file becomes a different
    /// objective's answer and this test is the only thing that would say so.
    func testTheForceScaleIsExactlyFmaxAtTheseLengths() throws {
        let pose = try XCTUnwrap(Self.poseOrder.first)
        let model = try XCTUnwrap(Self.build(pose: pose, source: .ours))
        let tau = try XCTUnwrap(Self.torques(pose: pose, reference: .analytic, activation: 0.9,
                                             shape: [Double](repeating: 0.8, count: 6)))
        let result = try XCTUnwrap(MuscleSolver().solveReal(
            withJointTorques: tau.map(NSNumber.init(value:)),
            momentArms: model.arms.map(NSNumber.init(value:)),
            muscleNames: model.names,
            muscleLengths: model.lengths.map(NSNumber.init(value:)),
            maxForces: model.maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: model.optimalFiberLengths.map(NSNumber.init(value:)),
            tendonSlackLengths: model.tendonSlackLengths.map(NSNumber.init(value:)),
            pennationAngles: model.names.map { _ in NSNumber(value: 0.0) },
            jointVelocities: Self.dofNames.map { _ in NSNumber(value: 0.0) },
            dofNames: Self.dofNames, dt: 1.0 / 30.0, softPenalty: 100.0))
        var worst = 0.0
        for (index, name) in result.muscleNames.enumerated() {
            let a = result.activations[index].doubleValue
            let f = result.forces[index].doubleValue
            guard a > 0, let row = model.names.firstIndex(of: name) else { continue }
            worst = Swift.max(worst, abs(f / a - model.maxForces[row]) / model.maxForces[row])
        }
        print("LEAK-METRIC force_scale worst_relative_departure_from_Fmax=\(worst)")
        XCTAssertLessThan(worst, 1e-9,
                          "at l_opt + l_Ts with zero pennation and zero velocity the force scale "
                          + "must be exactly F_max, or BoxQP is solving a different objective")
    }

    /// **The exact solver is checked against the shipping one where they must
    /// agree**, and its own KKT conditions are checked everywhere. Two solvers
    /// that never agree are two bugs; two that agree to OSQP's tolerance are one
    /// instrument and one measurement.
    func testTheExactSolverSatisfiesKKTAndAgreesWithOSQPToItsOwnTolerance() throws {
        let cells = Self.sweep()
        let readable = Self.readable(.ours, in: cells)
        let all = cells.filter { $0.subject == .ours }
        let worstKKT = all.map(\.kktResidual).max() ?? .nan
        let slacks = readable.map(\.solverSlack)
        print("LEAK-METRIC exact_solver worst_kkt_residual=\(worstKKT) "
              + "cells=\(all.count) readable=\(readable.count) "
              + "median_solver_slack_pp=\(WrapValidationHarness.percentile(slacks, 0.5)) "
              + "p90=\(WrapValidationHarness.percentile(slacks, 0.9)) "
              + "max_solver_slack_pp=\(slacks.max() ?? 0) "
              + "median_torque_residual=\(WrapValidationHarness.percentile(all.map(\.torqueResidual), 0.5)) "
              + "median_activation=\(WrapValidationHarness.percentile(readable.map(\.medianActivation), 0.5))")
        XCTAssertLessThan(worstKKT, 1e-6,
                          "every exact solve must reach a KKT point, or the instrument is not one")
        XCTAssertGreaterThanOrEqual(readable.count, Self.minimumCells,
                                    "R6: at least \(Self.minimumCells) readable cells")
    }

    // MARK: - R4: every path is modelled, and no displayed muscle is not

    /// **How many muscles still have unmodelled paths, which ones, and whether
    /// any of them is one the product names.** Read from the parser's own
    /// fidelity report on BOTH shipped models, never from the hand-written table
    /// — the table is checked against this, not the other way round.
    func testNoMuscleTheProductNamesHasAnUnmodelledPath() throws {
        var unmodelled: [String] = []
        let fullBody = try XCTUnwrap(WrapValidationHarness.fullBodyReport)
        print("LEAK-METRIC unmodelled model=FullBody solved=\(fullBody.solvedPathWraps) "
              + "unmodelled=\(fullBody.unmodelledPathWraps) "
              + "muscles=\(fullBody.musclesWithUnmodelledPathWraps)")
        XCTAssertEqual(fullBody.unmodelledPathWraps, 0,
                       "FullBody.osim still has unsolved PathWrap references: "
                       + "\(fullBody.musclesWithUnmodelledPathWraps)")
        unmodelled += fullBody.musclesWithUnmodelledPathWraps

        let bundle = Bundle(for: type(of: self))
        let path = try XCTUnwrap(bundle.path(forResource: "Rajagopal2016", ofType: "osim"))
        let bridge = NimbleBridge()
        XCTAssertTrue(bridge.loadModel(fromPath: path))
        let computer = MomentArmComputer()
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))
        let report = computer.fidelityReport
        print("LEAK-METRIC unmodelled model=Rajagopal2016 solved=\(report.solvedPathWraps) "
              + "unmodelled=\(report.unmodelledPathWraps) "
              + "muscles=\(report.musclesWithUnmodelledPathWraps)")
        XCTAssertEqual(report.unmodelledPathWraps, 0)
        unmodelled += report.musclesWithUnmodelledPathWraps

        let named = Set(unmodelled).compactMap { GaitLoadSummary.split($0)?.base }
                                   .filter { GaitLoadSummary.displayNames[$0] != nil }
        XCTAssertTrue(named.isEmpty,
                      "muscles the product names still have unmodelled paths: \(named)")
        XCTAssertEqual(GaitLoadSummary.musclesWithUnmodelledPaths, [],
                       "the shipped table must agree with the parser")
    }

    // MARK: - R5: the control, which is what makes a small leak mean something

    /// **The BEFORE.** The same rig, the same torques, the same reference —
    /// driven with the straight-line moment arms this project shipped until
    /// 2026-08-08. If this does not leak, the rig cannot see a moment-arm error
    /// at all and nothing else in this file is evidence.
    func testTheStraightLineArmsLeakMoreThanThePinnedClipsCanResolve() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let readable = Self.readable(.straightLine, in: cells)
        let worst = try XCTUnwrap(readable.max { $0.leakExact < $1.leakExact },
                                  "the control produced no readable cell")
        print("LEAK-METRIC control straight_line worst_exact_leak_pp=\(worst.leakExact) "
              + "at pose=\(worst.pose) ref=\(worst.reference.rawValue) shape=\(worst.shape) "
              + "effort=\(worst.effort) on=\(worst.worstBase) screened=\(worst.screened) "
              + "worst_shipped_leak_pp=\(readable.map(\.leakShipped).max() ?? 0) "
              + "named_leak_pp=\(readable.map(\.leakAmongNamed).max() ?? 0) "
              + "median_exact_leak_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakExact), 0.5)) "
              + "cells_over_floor=\(readable.filter { $0.leakExact > floor }.count)"
              + "/\(readable.count) floor_percent=\(floor)")
        XCTAssertGreaterThan(worst.leakExact, floor,
                             "the straight-line arms must leak more than the finest pinned clip "
                             + "can resolve, or this rig is not sensitive enough to certify "
                             + "anything about the wrapped ones")
    }

    // MARK: - THE MEASUREMENT

    /// **R1, R2 and R6.** The leak this build's own moment arms produce, over
    /// every leg pose in the sweep, both OpenSim reference definitions, five
    /// torque shapes and three effort levels — separated into the moment-arm
    /// cause and the solver cause, and reported against the floor read from the
    /// pinned clips.
    /// **The verdict is asserted in `testTheShippedFlagMatchesWhatTheMeasurementSupports`
    /// and nowhere else.** This test measures and reports; what it ASSERTS is the
    /// gain the wrap work actually bought, as a regression tripwire with a number
    /// — because a test that asserted `leak < threshold` would be asserting a
    /// hypothesis, and this measurement's answer is no.
    func testTheWrappedMomentArmsLeakLessThanAFifthOfThePublicationFloor() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let readable = Self.readable(.ours, in: cells)
        let control = Self.readable(.straightLine, in: cells)
        let worstExact = try XCTUnwrap(readable.max { $0.leakExact < $1.leakExact },
                                       "no readable cell — the sweep measured nothing")
        let worstShipped = try XCTUnwrap(readable.max { $0.leakShipped < $1.leakShipped })
        let worstNamed = try XCTUnwrap(readable.max { $0.leakAmongNamed < $1.leakAmongNamed })
        let median = WrapValidationHarness.percentile(readable.map(\.leakExact), 0.5)
        let controlMedian = WrapValidationHarness.percentile(control.map(\.leakExact), 0.5)

        var perReference: [String] = []
        for reference in Self.ArmSource.allCases where reference.isReference {
            let subset = readable.filter { $0.reference == reference }
            let worst = subset.max { $0.leakExact < $1.leakExact }
            perReference.append(String(format:
                "%@ exact[max %.4f on %@ at %@ | p99 %.4f median %.4f] "
                + "shipped[max %.4f median %.4f] n=%d",
                reference.rawValue, subset.map(\.leakExact).max() ?? 0,
                worst?.worstBase ?? "-", worst?.pose ?? "-",
                WrapValidationHarness.percentile(subset.map(\.leakExact), 0.99),
                WrapValidationHarness.percentile(subset.map(\.leakExact), 0.5),
                subset.map(\.leakShipped).max() ?? 0,
                WrapValidationHarness.percentile(subset.map(\.leakShipped), 0.5), subset.count))
        }
        print("LEAK-METRIC wrapped worst_exact_leak_pp=\(worstExact.leakExact) "
              + "at pose=\(worstExact.pose) ref=\(worstExact.reference.rawValue) "
              + "shape=\(worstExact.shape) effort=\(worstExact.effort) "
              + "on=\(worstExact.worstBase) screened=\(worstExact.screened) | "
              + "worst_shipped_leak_pp=\(worstShipped.leakShipped) "
              + "at pose=\(worstShipped.pose) ref=\(worstShipped.reference.rawValue) "
              + "shape=\(worstShipped.shape) effort=\(worstShipped.effort) "
              + "on=\(worstShipped.worstBase) solver_slack_there_pp=\(worstShipped.solverSlack) "
              + "exact_leak_there_pp=\(worstShipped.leakExact) | "
              + "worst_named_shipped_pp=\(worstNamed.leakAmongNamed) | "
              + "median_exact_pp=\(median) control_median_exact_pp=\(controlMedian) "
              + "p99_exact_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakExact), 0.99)) "
              + "median_shipped_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.leakShipped), 0.5)) "
              + "cells=\(readable.count) per_reference=\(perReference) "
              + "floor_percent=\(floor) threshold_pp=\(threshold) "
              + "R1_pass=\(worstExact.leakExact < threshold) "
              + "R2_pass=\(worstShipped.leakShipped < threshold)")

        XCTAssertGreaterThanOrEqual(worstExact.screened, Self.minimumScreenedBases,
                                    "R6: the maximum must be over a population")
        XCTAssertLessThan(median, controlMedian / 3,
                          "the wrap solver has to have bought something measurable: median "
                          + "moment-arm leak \(median) pp against the straight line's "
                          + "\(controlMedian) pp")
        XCTAssertLessThan(median, threshold,
                          "and the TYPICAL cell has to be inside the reopening threshold, or the "
                          + "wrap work did not move the distribution at all")
    }

    /// **THE BINDING CONSTRAINT, and it is not the moment arms.** `MuscleSolver`
    /// runs OSQP at `eps_abs = eps_rel = 1e-3`, polishing off, and accepts
    /// `OSQP_SOLVED_INACCURATE` — ten times looser again. On this rig that is
    /// worth more of a published left/right percentage than the entire
    /// moment-arm error, and more than the finest pinned clip can resolve.
    ///
    /// Measured with the geometry held FIXED: the same arms, the same torques,
    /// OSQP's answer against the exact minimiser of the same objective. No
    /// reference model is involved, so no disagreement between OpenSim's two
    /// columns can explain it.
    ///
    /// **This assertion is in the direction the defect points**, like
    /// `MomentArmErrorCancellationTests.testASignFlippedMomentArmPinsTheMuscleToTheFloorAndReadsFalselyEven`.
    /// If it ever fails, the solver has been tightened and the whole per-muscle
    /// decision has to be re-read — do not delete it, re-run the decision.
    func testTheShippingSolversOwnSlackIsLargerThanThePublicationFloor() throws {
        let readable = Self.readable(.ours, in: Self.sweep())
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let slacks = readable.map(\.solverSlack)
        let median = WrapValidationHarness.percentile(slacks, 0.5)
        let activation = WrapValidationHarness.percentile(readable.map(\.medianActivation), 0.5)
        // The mechanism, in one line: an ABSOLUTE tolerance on `a` becomes a
        // RELATIVE error in `100·(a_l − a_r)/mean`.
        let predicted = 100 * 2 * MuscleSolver.saturationActivationTolerance / activation
        print("LEAK-METRIC solver_slack median_pp=\(median) "
              + "p90_pp=\(WrapValidationHarness.percentile(slacks, 0.9)) "
              + "max_pp=\(slacks.max() ?? 0) median_activation=\(activation) "
              + "predicted_from_tolerance_pp=\(predicted) "
              + "osqp_accepted_tolerance=\(MuscleSolver.saturationActivationTolerance) "
              + "floor_percent=\(floor) cells=\(readable.count)")
        XCTAssertGreaterThan(median, floor,
                             "the shipping solver's own termination slack moves a published "
                             + "left/right figure by \(median) pp, against a \(floor) % floor — "
                             + "so no per-muscle number can be published at this tolerance "
                             + "however good the moment arms are")
    }

    /// **R7 — the claim has to be informative as well as safe.** The retirement's
    /// second half was that where the error cancels, every muscle reads the same
    /// number. Measure both quantities on the same solves: the SPREAD of the true
    /// left/right figures across muscles, and the error in them.
    func testThePerMuscleDifferencesAreLargerThanTheErrorInThem() throws {
        let readable = Self.readable(.ours, in: Self.sweep())
        func ratio(_ error: (Cell) -> Double) -> Double {
            WrapValidationHarness.percentile(
                readable.map { error($0) > 0 ? $0.trueSpread / error($0) : Double.infinity }
                        .filter { $0.isFinite }, 0.5)
        }
        let shipped = ratio(\.leakShipped)
        let exact = ratio(\.leakExact)
        let worstCell = readable.min {
            ($0.leakShipped > 0 ? $0.trueSpread / $0.leakShipped : .infinity)
                < ($1.leakShipped > 0 ? $1.trueSpread / $1.leakShipped : .infinity)
        }
        print("LEAK-METRIC information median_spread_over_shipped_error=\(shipped) "
              + "median_spread_over_moment_arm_error=\(exact) "
              + "min_at pose=\(worstCell?.pose ?? "-") shape=\(worstCell?.shape ?? "-") "
              + "spread_pp=\(worstCell?.trueSpread ?? 0) "
              + "shipped_error_pp=\(worstCell?.leakShipped ?? 0) median_spread_pp="
              + "\(WrapValidationHarness.percentile(readable.map(\.trueSpread), 0.5)) "
              + "cells=\(readable.count) "
              + "R7_pass=\(shipped > Self.minimumInformationToLeakRatio)")
        XCTAssertGreaterThanOrEqual(readable.count, Self.minimumCells)
        // The MOMENT-ARM half of R7 is the half this stage's work could move, and
        // it is asserted. The shipped half is reported and consumed by the
        // decision test, which is where the verdict lives.
        XCTAssertGreaterThan(exact, Self.minimumInformationToLeakRatio,
                             "with the arms this build computes, the per-muscle differences the "
                             + "panel would print must be larger than the moment-arm error in "
                             + "them")
    }

    // MARK: - The decision, pinned to the flag

    /// **The tripwire that keeps the shipped flag and this measurement in the
    /// same state.** It states the decision in one place: if the gates above
    /// pass, `perMuscleLeftRightClaimIsSupported` must be true; if any fails, it
    /// must be false. Weakening a gate to move the flag therefore breaks this
    /// test as well as that one.
    func testTheShippedFlagMatchesWhatTheMeasurementSupports() throws {
        let cells = Self.sweep()
        let floor = try smallestPublicationFloorOnThePinnedClips()
        let threshold = floor * Self.reopenFractionOfFloor
        let readable = Self.readable(.ours, in: cells)
        let control = Self.readable(.straightLine, in: cells)
        let worstExact = readable.map(\.leakExact).max() ?? .infinity
        let worstShipped = readable.map(\.leakShipped).max() ?? .infinity
        let ratios = readable.map {
            $0.leakShipped > 0 ? $0.trueSpread / $0.leakShipped : Double.infinity
        }.filter { $0.isFinite }
        let ratio = WrapValidationHarness.percentile(ratios, 0.5)

        let supported = worstExact < threshold                                   // R1
            && worstShipped < threshold                                          // R2
            && GaitLoadSummary.musclesWithUnmodelledPaths.isEmpty                // R4
            && (control.map(\.leakExact).max() ?? 0) > floor                     // R5
            && readable.count >= Self.minimumCells                               // R6
            && ratio > Self.minimumInformationToLeakRatio                        // R7
        print("LEAK-METRIC decision supported=\(supported) exact_pp=\(worstExact) "
              + "shipped_pp=\(worstShipped) threshold_pp=\(threshold) "
              + "control_pp=\(control.map(\.leakExact).max() ?? 0) cells=\(readable.count) "
              + "median_spread_over_error=\(ratio) "
              + "shipped_flag=\(GaitLoadSummary.perMuscleLeftRightClaimIsSupported)")
        XCTAssertEqual(GaitLoadSummary.perMuscleLeftRightClaimIsSupported, supported,
                       "the shipped flag and the measurement have diverged: moment arms "
                       + "\(worstExact) pp, printed number \(worstShipped) pp, threshold "
                       + "\(threshold) pp")
    }
}
