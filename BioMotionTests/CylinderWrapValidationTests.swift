import XCTest
@testable import BioMotion

/// Does the ported cylinder wrap solver reproduce OpenSim's moment arms?
///
/// # ORIGINAL PRE-REGISTRATION (historical; A4 below is current)
///
/// Written before a single number was read, and not edited afterwards. The
/// reference is `BioMotionTests/Fixtures/opensim_moment_arms.txt` — OpenSim 4.6
/// reading the SAME `FullBody.osim`, wrapping solved, 173 poses.
///
/// Muscles are split three ways, by the parser's OWN report rather than by a
/// hand-written list, so the split cannot drift from what the code does:
///
/// - **SOLVED** — carries a `PathWrap` and none of them are unmodelled. Every
///   wrap is a `WrapCylinder` this build solves. 56 muscles. **The claim.**
/// - **PARTIAL/UNSOLVED** — carries at least one unmodelled wrap. Empty on this
///   model since the ellipsoid landed; the class stays because a future model,
///   or a `<method>` other than `hybrid`, puts muscles back into it.
/// - **NO WRAP** — carries no `PathWrap`. 454 muscles. The CONTROL: the two
///   reference columns are identical there to the last stored digit, so any
///   change is this stage breaking something it was not supposed to touch.
///
/// The 10 muscles that carry a `WrapEllipsoid` are EXCLUDED from every number
/// in this file (`surface != .carriesEllipsoid`) and gated separately in
/// `EllipsoidWrapValidationTests`. Two solvers averaged together is one
/// solver's residual hiding inside the other's.
///
/// ## CORRECT — all of these must hold
///
/// - **C1** `max |ours − reference| ≤ 5.0 mm` on SOLVED muscles. The bar is the
///   straight-line implementation's own residual against ITS matching column,
///   4.39 mm (`StraightLinePathErrorTests`, 2026-08-08): what remains after a
///   faithful wrap solver is the same FK / spline / finite-difference gap that
///   was already there, not a new one.
/// - **C2** `p99 |ours − reference| ≤ 4.0 mm` on SOLVED muscles (was 3.79 mm).
/// - **C3** zero sign disagreements on SOLVED muscles where `|reference| ≥ 1 mm`
///   (restored as the fixed regression tripwire by A4).
/// - **C4** the CONTROL is unchanged: `max |ours − reference| ≤ 5.0 mm` on
///   muscles with no `PathWrap`, which is where it already was.
/// - **C5** path LENGTH: `max |ours − reference length| ≤ 5.0 mm` on SOLVED.
/// - **C6** wrap ENGAGEMENT agrees with OpenSim's `wrapPoints` on ≥ 99 % of
///   (pose, SOLVED muscle) rows.
/// - **C7** the fidelity report counts solved and unmodelled wraps separately,
///   and its numbers are the ones the display layer reads. It said 64 solved /
///   12 unmodelled while only the cylinder shipped; since the ellipsoid landed
///   it says 76 / 0. Rajagopal2016 is 46 / 0 either way (all cylinders).
///
/// ## WRONG — any one of these means the port is wrong, not "close enough"
///
/// - **W1** `max |ours − reference| > 20 mm` on a SOLVED muscle. Four times the
///   known implementation residual is not FK noise; it is a different branch.
/// - **W2** any sign flip against a reference `≥ 1 mm` on a SOLVED muscle. This
///   is what a backwards `quadrant` looks like, and it is the failure mode that
///   produces a path that renders beautifully on the wrong side of the bone.
/// - **W3** the median error on SOLVED muscles is not strictly better than the
///   straight line's. Wrapping that does not help is wrapping that is wrong.
/// - **W4** any muscle with no `PathWrap` moves at all.
/// - **W5** a non-finite number anywhere in the matrix.
///
/// A result between 5 mm and 20 mm is neither: it means the port is close and
/// something is still off, and it gets NAMED (muscle, coordinate, pose) rather
/// than averaged away.
///
/// # AMENDMENTS, and the evidence for each
///
/// The first run failed C1 (8.07 mm), C5 (8.75 mm) and C3/W2 (4 sign flips).
/// A1 and A2 traced those results to properties of the REFERENCE rather than of
/// the port. A3 later recorded a runtime threshold that moved when an unrelated
/// SimmSpline defect was fixed; A4 retires that self-calibration and restores
/// the original fixed 1 mm regression tripwire. Each amendment records the
/// evidence and timing that led to it.
///
/// **A1 — the analytic column is not `dL/dq`.** `GeometryPath::computeMomentArm`
/// asks `MomentArmSolver` for the generalized force a unit tension along the
/// current path produces, with the wrap points held fixed on their bodies. That
/// equals `−dL/dq` where the path varies smoothly with q; where the wrap
/// solution is marginal it does not. Measured, with OpenSim differencing its OWN
/// length at the same `eps = 1e-4` (`tools/opensim_ref/fd_check.py`):
///
///     TR2_l   / L2_L3_FE    at spine_flexed: analytic +0.002252, central −0.005274
///     TR2_l   / L3_L4_LB    at spine_flexed: analytic +0.001947, central −0.000490
///     gasmed_r/ knee_angle_r at neutral:     analytic +0.021761, central +0.004891
///     gasmed_r/ knee_angle_r at squat_deep:  analytic +0.026091, central +0.026090
///
/// The shipped values at those four pairs are −0.00527, −0.00049, … — i.e. this
/// code already reproduces OpenSim's own central difference to the micron, and
/// all four "sign flips" are the analytic-versus-central gap. So the moment-arm
/// gates move to `OpenSimFiniteDifferenceFixture`, which is the same quantity
/// this code computes. The analytic comparison stays, reported and gated only at
/// W1/W3, because it is still the number a reader of OpenSim would quote.
///
/// **A2 — muscles with two or more `PathWrap`s take OpenSim's ITERATIVE path,
/// and its output is not self-consistent.** With one wrap OpenSim solves the
/// spiral in closed form; with two or more it re-solves the whole set up to 8
/// times. On `gasmed_r` at `neutral` the path OpenSim reports has tangent points
/// that are the closed-form solution for the ORIGINAL `P1→P2` segment (verified:
/// deactivating the other wrap object reproduces them to 6 dp) while its stored
/// spiral length, 0.038054, belongs to a later `C2→P2` solve — the spiral
/// implied by its own tangent points is 0.046516. This port re-solves to a
/// fixed point and is self-consistent, so it cannot match that state; it differs
/// by 8.75 mm of length on the 4 two-cylinder muscles (`gasmed`, `gaslat140`,
/// left and right) and matches everywhere else.
///
/// The gates therefore stratify on a property of the MODEL — how many
/// `PathWrap`s a muscle carries — not on which muscles happened to disagree:
///
/// - **SINGLE-WRAP SOLVED** (52 muscles): full pre-registered thresholds.
/// - **MULTI-WRAP SOLVED** (4 muscles): reported with their numbers, gated only
///   at W1/W3. Recorded in STATUS.md as open.
final class CylinderWrapValidationTests: XCTestCase {

    typealias Sample = WrapValidationHarness.Sample
    typealias LengthSample = WrapValidationHarness.LengthSample

    /// Every number below comes from `WrapValidationHarness`, restricted to the
    /// muscles whose every `PathWrap` names a `WrapCylinder`. Muscles carrying a
    /// `WrapEllipsoid` are a different solver with a different tolerance and are
    /// gated in `EllipsoidWrapValidationTests`; mixing them here would let one
    /// solver's residual hide inside the other's average.
    private static var samples: [Sample] {
        WrapValidationHarness.samples.filter { $0.surface != .carriesEllipsoid }
    }
    private static var lengthSamples: [LengthSample] {
        WrapValidationHarness.lengthSamples.filter { $0.surface != .carriesEllipsoid }
    }
    private static var cylinderMuscles: Set<String> {
        WrapValidationHarness.solvedMuscles.subtracting(WrapValidationHarness.ellipsoidMuscles)
    }

    override func setUpWithError() throws {
        try WrapValidationHarness.requireBuild(bundle: Bundle(for: type(of: self)))
    }

    // MARK: - Did anything get measured

    func testTheThreeMuscleClassesAreNonEmptyAndDisjoint() {
        XCTAssertGreaterThan(Self.samples.count, 1000,
                             "nothing was measured, so every number below is vacuous")
        let solved = Self.samples.filter { $0.wrapClass == .solved }
        let unsolved = Self.samples.filter { $0.wrapClass == .unsolved }
        let none = Self.samples.filter { $0.wrapClass == .none }
        print("WRAP-CLASSES cylinder-only=\(Self.cylinderMuscles.count) muscles / \(solved.count) pairs, "
              + "unsolved=\(WrapValidationHarness.unsolvedMuscles.count) muscles / \(unsolved.count) pairs, "
              + "carries-ellipsoid=\(WrapValidationHarness.ellipsoidMuscles.count) muscles "
              + "(gated in EllipsoidWrapValidationTests), no-wrap pairs=\(none.count)")
        XCTAssertGreaterThan(solved.count, 0, "no muscle's wraps are solved — nothing shipped")
        XCTAssertGreaterThan(none.count, 0, "the control class is empty")
        XCTAssertEqual(Self.cylinderMuscles.count, 56,
                       "FullBody.osim has 56 muscles whose every PathWrap is a WrapCylinder")
        XCTAssertEqual(WrapValidationHarness.ellipsoidMuscles.count, 10,
                       "and 10 that carry a WrapEllipsoid: "
                       + "\(WrapValidationHarness.ellipsoidMuscles.sorted())")
        XCTAssertEqual(unsolved.count, 0,
                       "a cylinder-only muscle whose wraps are not solved would mean the "
                       + "parser rejected a WrapCylinder this suite thinks it is claiming")
    }

    // MARK: - C7 / the fidelity report

    func testTheFidelityReportCountsSolvedAndUnmodelledWrapsSeparately() throws {
        let report = try XCTUnwrap(WrapValidationHarness.fullBodyReport)
        print("FIDELITY \(report.summary)")
        XCTAssertEqual(report.solvedPathWraps, 76,
                       "all 76 of FullBody's PathWrap references are solved: 64 WrapCylinder "
                       + "since 2026-08-08 and the 12 WrapEllipsoid added by this stage")
        XCTAssertEqual(report.unmodelledPathWraps, 0,
                       "nothing is left unmodelled in this model. The count is not decoration: "
                       + "GaitLoadSummary.musclesWithUnmodelledPaths reads it and the display "
                       + "decides what may be compared with what from it")
        XCTAssertEqual(report.wrapObjectsParsed, 69)
        XCTAssertEqual(report.wrapObjectsRejected, 0,
                       "every WrapObject in the shipped model must parse, or a PathWrap "
                       + "pointing at it silently stops wrapping")
    }

    // MARK: - W5

    func testEveryMomentArmIsFinite() {
        let bad = Self.samples.filter { !$0.ours.isFinite }
        XCTAssertEqual(bad.count, 0,
                       "non-finite moment arms: \(bad.prefix(5).map { "\($0.muscle)/\($0.coordinate)" })")
    }

    // MARK: - C4 / W4: the control

    /// Muscles with no `PathWrap` are identical in OpenSim's two columns to the
    /// last stored digit. If this stage moved one of them, the driver is running
    /// on paths it has no business touching.
    func testMusclesWithNoWrapObjectAreUnchanged() {
        let control = Self.samples.filter { $0.wrapClass == .none }
        let identical = control.filter { $0.wrapOn == $0.wrapOff }
        XCTAssertEqual(identical.count, control.count,
                       "the fixture's own control is broken: \(control.count - identical.count) "
                       + "no-wrap pairs differ between its two columns")
        let errors = control.map { abs($0.ours - $0.wrapOn) }
        print(WrapValidationHarness.describe(errors, label: "CONTROL: no-wrap muscles, ours vs reference"))
        XCTAssertLessThan(errors.max() ?? .infinity, 0.005,
                          "C4: a muscle with no wrap object must be exactly where it was")
    }

    // MARK: - C1 / C2 / C3 / W1 / W2 / W3: the claim

    /// **C1 / C2, on the definition-matched column (amendment A1) and the
    /// closed-form muscles (amendment A2).**
    ///
    /// # W1's multi-wrap component is a tripwire, not a gate — read this before
    /// # adding poses
    ///
    /// `allErrors` mixes the single-wrap and multi-wrap classes, and for the
    /// four gastrocnemius muscles the column it is measured against **is not a
    /// path length**. `calcLengthAfterPathComputation` adds straight segments
    /// measured between the wrap points OpenSim REPORTS to the spiral length
    /// OpenSim STORED beside them, and for a two-cylinder path those halves
    /// describe different paths: on `gasmed_r` at knee 0 deg the stored spiral
    /// is 0.038054 m while the chord between the two points it is supposed to
    /// connect is 0.045350 m, i.e. the reported total is 7.30 mm below the
    /// length of any path through its own points (`WrapCylinder::_adjust_
    /// tangent_point` moves the tangent points and no one recomputes the arc).
    ///
    /// So this bar passes for the multi-wrap class at 15.8 mm because of where
    /// the committed 5-deg grid lands, not because 20 mm bounds anything. Scan
    /// `knee_angle_r` at a step finer than the centred-difference stencil and
    /// `gaslat140_r` reads **41.26 mm at 26.01866 deg** — while the same pose,
    /// same port, against a reconciled column reads **0.54 mm**.
    ///
    /// Adding poses to `tools/opensim_ref/poses.py` can therefore fail this
    /// assertion for a reason that is not this port's. The gate for that class
    /// is `MultiWrapReferenceTests`, which holds it to C1/C2/C5's own bars on a
    /// grid that deliberately includes the reference's worst rows. This one is
    /// left as written — a change of BRANCH would move it by metres, which is
    /// still worth catching end-to-end through nimble's forward kinematics.
    func testSingleWrapMusclesMatchOpenSimsOwnDerivative() {
        let solved = Self.samples.filter { $0.wrapClass == .solved && $0.centralDifference != nil }
        let single = solved.filter { $0.wrapCount == 1 }
        let multi = solved.filter { $0.wrapCount > 1 }
        XCTAssertGreaterThan(single.count, 0, "no single-wrap samples were collected")
        XCTAssertGreaterThan(multi.count, 0, "no multi-wrap samples were collected")

        let singleErrors = single.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        let multiErrors = multi.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        let allErrors = solved.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        print(WrapValidationHarness.describe(singleErrors, label: "SINGLE-WRAP: ours vs OpenSim central difference"))
        print(WrapValidationHarness.describe(multiErrors, label: "MULTI-WRAP: ours vs OpenSim central difference"))
        print(WrapValidationHarness.describe(allErrors, label: "ALL SOLVED: ours vs OpenSim central difference"))
        print(WrapValidationHarness.worstOffenders(in: single, by: { abs($0.ours - ($0.centralDifference ?? 0)) },
                                  label: "largest SINGLE-WRAP residuals"))
        print(WrapValidationHarness.worstOffenders(in: multi, by: { abs($0.ours - ($0.centralDifference ?? 0)) },
                                  label: "largest MULTI-WRAP residuals"))

        XCTAssertLessThan(singleErrors.max() ?? .infinity, 0.005,
                          "C1: max residual over single-wrap solved muscles")
        XCTAssertLessThan(WrapValidationHarness.percentile(singleErrors, 0.99), 0.004,
                          "C2: p99 residual over single-wrap solved muscles")
        XCTAssertLessThan(allErrors.max() ?? .infinity, 0.020,
                          "W1: a >20 mm residual anywhere is a different branch, not FK noise")
    }

    /// **W3, and the whole point of the change**: how much of the straight
    /// line's error the wrap solver removes, against the reference a reader of
    /// OpenSim would quote. Gated only at W1/W3 — see amendment A1 for why the
    /// tight thresholds do not apply to this column.
    func testWrappingBeatsTheStraightLineAgainstTheAnalyticReference() {
        let solved = Self.samples.filter { $0.wrapClass == .solved }
        let errors = solved.map { abs($0.ours - $0.wrapOn) }
        let straightLine = solved.map { abs($0.wrapOff - $0.wrapOn) }
        print(WrapValidationHarness.describe(errors, label: "SOLVED: ours vs analytic reference"))
        print(WrapValidationHarness.describe(straightLine, label: "SOLVED: straight line vs analytic reference"))
        print(WrapValidationHarness.worstOffenders(in: solved, by: { abs($0.ours - $0.wrapOn) },
                                  label: "largest residuals vs the ANALYTIC column"))
        let median = WrapValidationHarness.percentile(errors, 0.5)
        let medianBefore = WrapValidationHarness.percentile(straightLine, 0.5)
        XCTAssertLessThan(median, medianBefore,
                          "W3: wrapping must beat the straight line it replaced "
                          + "(median \(median) vs \(medianBefore))")
        XCTAssertLessThan(WrapValidationHarness.percentile(errors, 0.9), WrapValidationHarness.percentile(straightLine, 0.9),
                          "W3: and at p90, where the straight line's error lives")
    }

    /// **C3 / W2 — the backwards-`quadrant` detector**, on the
    /// definition-matched column. A sign flip against OpenSim's own derivative
    /// means the path ran round the other side of the bone.
    ///
    /// # AMENDMENT A3 (historical): the inclusion threshold was measured
    ///
    /// The pre-registration tested the sign wherever `|reference| ≥ 1 mm`. A3
    /// replaced that with the largest no-wrap disagreement measured in the same
    /// run: then-linearly-interpolated `MovingPathPoint` splines, Nimble FK and
    /// finite differencing put the implementations up to **3.758 mm** apart.
    ///
    /// That runtime threshold later proved to be coupled to an unrelated bug:
    /// correcting SimmSpline endpoint extrapolation moved the same no-wrap
    /// maximum from 3.758 mm to near machine precision without changing a
    /// cylinder path. A correct fix elsewhere must not silently redefine C3.
    ///
    /// # AMENDMENT A4: restore the original fixed sign tripwire
    ///
    /// FullBody now reports Moving `4 parsed / 0 approximated`, and the unrelated
    /// no-wrap maximum no longer supplies a meaningful runtime threshold. C3 is
    /// therefore again the original fixed 1 mm zero-flip regression on the
    /// definition-matched column. A failure below C1's 5 mm accuracy contract
    /// still needs attribution; it is not automatically blamed on the cylinder.
    /// The analytic 1 mm positive control and low-level quadrant tests remain.
    func testSolvedMusclesNeverPointTheWrongWay() {
        let signResolution = 0.001  // original C3/W2 contract
        let single = Self.samples.filter {
            $0.wrapClass == .solved && $0.wrapCount == 1
                && abs($0.centralDifference ?? 0) >= signResolution
        }
        let flipped = single.filter {
            ($0.ours < 0) != (($0.centralDifference ?? 0) < 0)
        }
        let oneToFive = single.filter {
            abs($0.centralDifference ?? 0) < 0.005
                && ($0.ours < 0) != (($0.centralDifference ?? 0) < 0)
        }
        let flippedMulti = Self.samples.filter {
            $0.wrapClass == .solved && $0.wrapCount > 1
                && abs($0.centralDifference ?? 0) >= signResolution
                && ($0.ours < 0) != (($0.centralDifference ?? 0) < 0)
        }
        // Against the analytic column too, so the amendment stays auditable.
        let analytic = Self.samples.filter { $0.wrapClass == .solved && abs($0.wrapOn) >= 0.001 }
        let flippedAnalytic = analytic.filter { ($0.ours < 0) != ($0.wrapOn < 0) }
        let flippedBefore = analytic.filter { ($0.wrapOff < 0) != ($0.wrapOn < 0) }
        print("SIGN resolution_mm=1 single=\(single.count) flipped=\(flipped.count) "
              + "one_to_five_mm=\(oneToFive.count) | "
              + "multi flipped=\(flippedMulti.count) | "
              + "vs ANALYTIC: pairs=\(analytic.count) flipped_now=\(flippedAnalytic.count) "
              + "flipped_by_the_straight_line=\(flippedBefore.count)")
        for sample in (flipped + flippedMulti + flippedAnalytic).prefix(10) {
            print(String(format: "  SIGN-FLIP %@ %@ %@ ours %+.5f central %+.5f analytic %+.5f",
                         sample.pose, sample.muscle, sample.coordinate, sample.ours,
                         sample.centralDifference ?? .nan, sample.wrapOn))
        }
        XCTAssertGreaterThan(single.count, 0)
        XCTAssertEqual(flipped.count, 0,
                       "W2/C3: a sign flip is what a backwards quadrant looks like")
        XCTAssertLessThan(flippedAnalytic.count, flippedBefore.count / 10,
                          "the straight line flipped \(flippedBefore.count) signs; wrapping "
                          + "must remove almost all of them even against the analytic column")
    }

    // MARK: - C5 / C6: length and engagement

    /// **C5.** Path length is definition-free — no `MomentArmSolver`, no finite
    /// difference — so it is the cleanest statement about the wrap solver
    /// itself. Stratified by amendment A2.
    func testSolvedMusclePathLengthsMatchTheReference() {
        let solved = Self.lengthSamples.filter { $0.wrapClass == .solved }
        let single = solved.filter { $0.wrapCount == 1 }
        let multi = solved.filter { $0.wrapCount > 1 }
        let singleErrors = single.map { abs($0.ours - $0.wrapOn) }
        let multiErrors = multi.map { abs($0.ours - $0.wrapOn) }
        print(WrapValidationHarness.describe(singleErrors, label: "SINGLE-WRAP: path length, ours vs reference"))
        print(WrapValidationHarness.describe(multiErrors, label: "MULTI-WRAP: path length, ours vs reference"))
        for sample in multi.sorted(by: { abs($0.ours - $0.wrapOn) > abs($1.ours - $1.wrapOn) })
                           .prefix(4) {
            print(String(format: "  LENGTH-MULTI %@ %@ ours %.6f ref %.6f",
                         sample.pose, sample.muscle, sample.ours, sample.wrapOn))
        }
        XCTAssertGreaterThan(single.count, 0)
        XCTAssertGreaterThan(multi.count, 0)
        XCTAssertLessThan(singleErrors.max() ?? .infinity, 0.005,
                          "C5: path length over single-wrap solved muscles")
        // Amendment A2 said the 4 two-cylinder muscles "cannot match OpenSim's
        // non-self-consistent iterate". Since 2026-08-09 that is measured rather
        // than asserted, and it is sharper than A2 guessed: the reference's
        // stored spiral arc is shorter than the CHORD between the two tangent
        // points it reports (worst -7.30 mm, 88 rows), so its total is not the
        // length of any path. Reconcile it with its own tangent points and this
        // class agrees to 0.045 mm — see `MultiWrapReferenceTests`, which is the
        // gate. The bar below stays where it was pre-registered; it now reads as
        // a bound on the REFERENCE's bookkeeping error at these poses, and the
        // note above `testSingleWrapMusclesMatchOpenSimsOwnDerivative` explains
        // why densifying the grid can push it through 20 mm without this port
        // changing at all.
        XCTAssertLessThan(multiErrors.max() ?? .infinity, 0.020,
                          "the 4 two-cylinder muscles cannot match OpenSim's non-self-consistent "
                          + "iterate (amendment A2), but they must stay inside W1's envelope")
    }

    /// Whether the wrap ENGAGED, not just how long it is. A solver that agrees
    /// on length while disagreeing about engagement is agreeing by accident.
    func testWrapEngagementAgreesWithOpenSim() {
        let solved = Self.lengthSamples.filter { $0.wrapClass == .solved }
        let agree = solved.filter { $0.ourWrapPoints == $0.referenceWrapPoints }
        let engagedInReference = solved.filter { $0.referenceWrapPoints > 0 }
        let engagedHere = solved.filter { $0.ourWrapPoints > 0 }
        let rate = Double(agree.count) / Double(max(1, solved.count))
        print(String(format: "ENGAGEMENT rows=%d agree=%d (%.2f%%) engaged_ref=%d engaged_ours=%d",
                     solved.count, agree.count, rate * 100,
                     engagedInReference.count, engagedHere.count))
        for sample in solved where sample.ourWrapPoints != sample.referenceWrapPoints {
            print("  ENGAGEMENT-DIFF \(sample.pose) \(sample.muscle) "
                  + "ours=\(sample.ourWrapPoints) ref=\(sample.referenceWrapPoints)")
            if sample.pose.hasPrefix("zzz") { break }
        }
        XCTAssertGreaterThan(engagedInReference.count, 0,
                             "the reference never engages a wrap at these poses, so this "
                             + "suite would pass with no solver at all")
        XCTAssertGreaterThanOrEqual(rate, 0.99, "C6: engagement agreement")
    }

    // MARK: - Cost

    /// The named risk. Separate Debug iOS Simulator receipts put moving-input
    /// warm-start IK (~6 mm/frame) at 1567 ms/frame (77.8 iterations) and the
    /// 520×109 QP at 194.4 ms; they are
    /// not additive end-to-end timings, and no Release-device timing exists. A
    /// wrap solver that iterates can still make the app unusable, so this test
    /// prints its own stage timing rather than borrowing either number. The one
    /// asserted ceiling means the feature cannot ship at all.
    func testMomentArmSolveCostPerFrame() {
        let timings = WrapValidationHarness.solveMilliseconds
        guard !timings.isEmpty else { return XCTFail("no timings were collected") }
        let sorted = timings.sorted()
        let mean = timings.reduce(0, +) / Double(timings.count)
        print(String(format: "MOMENT-ARM-COST n=%d mean %.1f ms  median %.1f  min %.1f  max %.1f "
                     + "(Debug, iOS Simulator, 169 coordinates x 520 muscles)",
                     timings.count, mean, sorted[sorted.count / 2], sorted[0],
                     sorted[sorted.count - 1]))
        let counters = WrapValidationHarness.discontinuityCounters
        let total = counters.centred + counters.oneSided + counters.unresolved
        print(String(format: "STENCIL total=%d centred=%d one-sided=%d (%.4f%%) unresolved=%d",
                     total, counters.centred, counters.oneSided,
                     100.0 * Double(counters.oneSided) / Double(max(1, total)),
                     counters.unresolved))
        XCTAssertGreaterThan(total, 0)
        XCTAssertLessThan(mean, 10_000,
                          "a moment-arm solve costing over ten seconds a frame is not a "
                          + "feature with a performance problem, it is not a feature")
    }

}
