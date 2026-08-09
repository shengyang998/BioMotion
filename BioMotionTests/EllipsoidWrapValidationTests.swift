import XCTest
@testable import BioMotion

/// Does the ported ellipsoid wrap solver reproduce OpenSim's moment arms, and
/// what does it cost?
///
/// # ORIGINAL PRE-REGISTRATION (historical; E-A3 below is current)
///
/// Written before a single number from this stage was read. The reference is
/// `BioMotionTests/Fixtures/opensim_moment_arms.txt` (OpenSim 4.6's analytic
/// column) and `opensim_moment_arms_fd.txt` (OpenSim's own central difference of
/// its own length — the definition this code computes, amendment A1 in
/// `CylinderWrapValidationTests`). Both read the SAME `FullBody.osim`.
///
/// The population is the 10 muscles that carry at least one `WrapEllipsoid`,
/// selected by `MomentArmComputer.ellipsoidPathWrapCount(forMuscleNamed:)` —
/// the parser's own answer, not a hand-written list. They are ANC, BIClong,
/// BICshort, BRD and TRIlong, left and right; all elbow. Eight wrap objects, 12
/// `PathWrap` references, all `<method>hybrid</method>`.
///
/// They stratify the same way the cylinder muscles do, because the reason is a
/// property of OpenSim rather than of the surface: with ONE `PathWrap` it solves
/// the path once, with two or more it re-solves the whole set up to 8 times and
/// its stored result is not internally self-consistent.
///
/// - **SINGLE-WRAP** — ANC, BICshort, BRD (6 muscles, one ellipsoid each).
///   Full thresholds. **The claim.**
/// - **MULTI-WRAP** — BIClong (2 ellipsoids) and TRIlong (2 cylinders + 1
///   ellipsoid). Reported with their numbers, gated only at the WRONG bar.
///
/// ## CORRECT — all of these must hold
///
/// - **E1** `max |ours − OpenSim's central difference| ≤ 5.0 mm` on SINGLE-WRAP
///   ellipsoid muscles. Same bar as the cylinder's C1, and for the same reason:
///   the straight-line implementation's own residual against its matching column
///   is 4.39 mm, so what survives a faithful solver is the FK / spline /
///   finite-difference gap that was already there.
/// - **E2** `p99 ≤ 4.0 mm` on the same population (cylinder C2's bar).
/// - **E3** zero sign disagreements on SINGLE-WRAP muscles above the CONTROL's
///   own measured disagreement floor. E-A3 replaces that moving threshold with
///   fixed effect and total-sign checks. A sign flip is what a backwards
///   `<quadrant>` looks like, and the ellipsoid has one on all 8 objects
///   (`x`, `-x`, `-y`, `z`).
/// - **E4** path LENGTH: `max |ours − reference| ≤ 5.0 mm` on SINGLE-WRAP.
/// - **E5** wrap ENGAGEMENT agrees with OpenSim's `wrapPoints` column on ≥ 99 %
///   of (pose, ellipsoid muscle) rows — single AND multi. Engagement is
///   discrete, so it is the one thing a length cannot agree with by accident,
///   and the ellipsoid engages only when the segment actually pierces it.
/// - **E6** `unmodelledPathWraps == 0` and `solvedPathWraps == 76` on FullBody.
/// - **E7** zero numerical refusals over the whole sweep. The port refuses
///   rather than returning the NaN OpenSim would in two places (DEVIATION 9); a
///   refusal that actually fires is a pose to report, not a rounding detail.
/// - **E8** the per-frame COST A/B, measured in one process at three poses with
///   the ellipsoids active and then deactivated, is under 3× the cylinder-only
///   cost. Beyond that the honest outcome is to ship the 56 cylinders and leave
///   the 8 ellipsoids counted and DISCLOSED.
///
/// ## WRONG — any one of these means the port is wrong, not "close enough"
///
/// - **X1** `max |ours − central difference| > 20 mm` on ANY ellipsoid muscle,
///   single or multi. Four times the known implementation residual is a
///   different branch, not FK noise.
/// - **X2** any sign flip above the control floor on a SINGLE-WRAP muscle
///   (historical wording; E-A3 is the current contract).
/// - **X3** the median error against the analytic column is not strictly better
///   than the straight line's. Wrapping that does not help is wrapping that is
///   wrong — and the straight line is measurably wrong here: 587 of 5,882 pairs
///   on these muscles have OpenSim's wrap-on and wrap-off columns in opposite
///   directions.
/// - **X4** a non-finite number anywhere.
/// - **X5** the cost A/B shows no difference at all, which would mean the
///   ellipsoid solver never ran and every number above is about the cylinders.
///
/// A result between 5 mm and 20 mm is neither: it gets NAMED (muscle,
/// coordinate, pose) rather than averaged away.
///
/// # AMENDMENTS, and the evidence for each
///
/// A1 and A2 were written before their amended numbers were read. A3 was added
/// after an unrelated SimmSpline fix exposed that the runtime-generated sign
/// threshold could move. None changes a threshold to cover a residual caused by
/// the ellipsoid — the A/B ablation is what establishes that attribution.
///
/// **E-A1 — E2's p99 threshold was copied from the wrong population.** 4.0 mm
/// was the CYLINDER muscles' measured p99. This population is different in a way
/// that was documented before this stage started: all four of `FullBody.osim`'s
/// `MovingPathPoint`s are on `BIClong_*` and `BICshort_*`, `MomentArmComputer`
/// then interpolated them LINEARLY between cubic-spline control points, and
/// STATUS.md recorded the resulting **4.39 mm** residual on
/// `BIClong_l`/`pro_sup_l` while only the cylinder shipped. E2 is E1's 5.0 mm.
/// The evidence that this was not
/// the ellipsoid's residual is the ablation, printed by
/// `testTheResidualThatSurvivesIsReportedWithoutPreAttribution`: the ten worst
/// survivors are all `BIClong_l` / `BICshort_l` about `pro_sup_l`, and each reads **4.414 mm with
/// the ellipsoid and 5.569 mm without it**. Turning the solver on made the
/// survivor smaller.
///
/// **E-A2 — X3 is measured by ABLATION, not against OpenSim's wrap-off column.**
/// As pre-registered, X3 compared `|ours − ref|` with `|wrap-off − ref|`. On this
/// population the wrap-off column's MEDIAN error is exactly 0 over this pose set,
/// so "strictly better than the median" was unreachable by construction; and that
/// column carries every other difference between the two implementations
/// (nimble's FK, the `MovingPathPoint` splines, the finite-difference step), which
/// on these muscles are millimetres. Subtracting two things that differ in two
/// ways attributes nothing. The replacement runs THIS code twice at the same
/// poses, with `setEllipsoidWrapObjectsActive(true)` and then `(false)`, and
/// requires the ellipsoid to move the pairs it changes towards OpenSim at both
/// the median and the maximum. The analytic-column comparison stays, reported.
///
/// **E-A3 — the sign gate measures the wrap's effect, not an unrelated error
/// maximum.** E3 originally used the largest no-wrap `ours − analytic` residual
/// from the current run as its inclusion threshold. Fixing SimmSpline endpoint
/// extrapolation correctly moved that unrelated control from 3.758 mm to near
/// machine precision and admitted 54 copies of the already-known BIC moving-
/// point residual. Ellipsoid length, engagement, analytic sign and every
/// residual were unchanged; only the runtime-generated threshold moved.
///
/// The replacement was registered before reading the exact-MovingPath result.
/// On the six-pose A/B, compare the causal effects
/// `(ellipsoids on − ellipsoids off)` and `(OpenSim wrap on − wrap off)` wherever
/// the reference effect clears the original fixed 1 mm sign resolution; a zero
/// actual effect fails rather than inheriting the positive sign. Across the
/// whole sweep, the original fixed 1 mm total-sign check remains a regression
/// tripwire. A failure below E1's 5 mm accuracy contract still needs attribution
/// and is not automatically blamed on the ellipsoid. The analytic 1 mm positive
/// control remains unchanged.
///
/// With exact MovingPath SimmSpline evaluation, FullBody reports Moving
/// `4 parsed / 0 approximated`. The affected sweep measures central-difference
/// max **2.679 mm** (formerly 4.414) and analytic max **2.301 mm** (formerly
/// 4.385); the fixed sign thresholds do not move with either result.
final class EllipsoidWrapValidationTests: XCTestCase {

    typealias Sample = WrapValidationHarness.Sample
    typealias LengthSample = WrapValidationHarness.LengthSample

    private static var samples: [Sample] {
        WrapValidationHarness.samples.filter { $0.surface == .carriesEllipsoid }
    }
    private static var lengthSamples: [LengthSample] {
        WrapValidationHarness.lengthSamples.filter { $0.surface == .carriesEllipsoid }
    }

    override func setUpWithError() throws {
        try WrapValidationHarness.requireBuild(bundle: Bundle(for: type(of: self)))
    }

    // MARK: - Did anything get measured

    func testTheEllipsoidPopulationIsWhatTheModelSays() {
        let muscles = WrapValidationHarness.ellipsoidMuscles
        print("ELLIPSOID-MUSCLES \(muscles.sorted())")
        XCTAssertEqual(muscles, ["ANC_l", "ANC_r", "BIClong_l", "BIClong_r",
                                 "BICshort_l", "BICshort_r", "BRD_l", "BRD_r",
                                 "TRIlong_l", "TRIlong_r"],
                       "the 10 muscles FullBody.osim gives a WrapEllipsoid")
        XCTAssertGreaterThan(Self.samples.count, 500,
                             "nothing was measured, so every number below is vacuous")
        let single = Self.samples.filter { $0.wrapCount == 1 }
        let multi = Self.samples.filter { $0.wrapCount > 1 }
        print("ELLIPSOID-STRATA single-wrap pairs=\(single.count) multi-wrap pairs=\(multi.count)")
        XCTAssertGreaterThan(single.count, 0)
        XCTAssertGreaterThan(multi.count, 0)
    }

    // MARK: - E6

    func testEveryEllipsoidPathWrapIsSolved() throws {
        let report = try XCTUnwrap(WrapValidationHarness.fullBodyReport)
        print("FIDELITY-ELLIPSOID \(report.summary)")
        XCTAssertEqual(report.solvedPathWraps, 76, "E6: 64 cylinder + 12 ellipsoid")
        XCTAssertEqual(report.unmodelledPathWraps, 0, "E6")
        XCTAssertEqual(report.musclesWithUnmodelledPathWraps.count, 0,
                       "E6: `GaitLoadSummary.musclesWithUnmodelledPaths` reads this list "
                       + "and the display decides from it what may be compared with what")
    }

    // MARK: - X4 / E7

    func testEveryEllipsoidMomentArmIsFiniteAndNothingRefused() {
        let bad = Self.samples.filter { !$0.ours.isFinite }
        XCTAssertEqual(bad.count, 0,
                       "X4: \(bad.prefix(5).map { "\($0.muscle)/\($0.coordinate)" })")
        print("NUMERICAL-REFUSALS \(WrapValidationHarness.ellipsoidNumericalRefusals) "
              + "over \(WrapValidationHarness.solveMilliseconds.count) poses")
        XCTAssertEqual(WrapValidationHarness.ellipsoidNumericalRefusals, 0,
                       "E7: the solver hit a case OpenSim answers with a NaN. It took the "
                       + "straight line there, which is defined but is not a wrap")
    }

    // MARK: - E1 / E2 / X1: the claim

    func testSingleWrapEllipsoidMusclesMatchOpenSimsOwnDerivative() {
        let withReference = Self.samples.filter { $0.centralDifference != nil }
        let single = withReference.filter { $0.wrapCount == 1 }
        let multi = withReference.filter { $0.wrapCount > 1 }
        XCTAssertGreaterThan(single.count, 0, "no single-wrap ellipsoid samples")
        XCTAssertGreaterThan(multi.count, 0, "no multi-wrap ellipsoid samples")

        let singleErrors = single.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        let multiErrors = multi.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        let allErrors = withReference.map { abs($0.ours - ($0.centralDifference ?? 0)) }
        print(WrapValidationHarness.describe(
            singleErrors, label: "ELLIPSOID SINGLE-WRAP: ours vs OpenSim central difference"))
        print(WrapValidationHarness.describe(
            multiErrors, label: "ELLIPSOID MULTI-WRAP: ours vs OpenSim central difference"))
        print(WrapValidationHarness.worstOffenders(
            in: single, by: { abs($0.ours - ($0.centralDifference ?? 0)) },
            label: "largest SINGLE-WRAP ellipsoid residuals"))
        print(WrapValidationHarness.worstOffenders(
            in: multi, by: { abs($0.ours - ($0.centralDifference ?? 0)) },
            label: "largest MULTI-WRAP ellipsoid residuals"))

        let worst = single.max(by: { abs($0.ours - ($0.centralDifference ?? 0))
                                     < abs($1.ours - ($1.centralDifference ?? 0)) })
        if let worst {
            print(String(format: "E1-WORST %@ %@ %@ ours %+.6f central %+.6f",
                         worst.pose, worst.muscle, worst.coordinate, worst.ours,
                         worst.centralDifference ?? .nan))
        }
        XCTAssertLessThan(singleErrors.max() ?? .infinity, 0.005, "E1")
        XCTAssertLessThan(WrapValidationHarness.percentile(singleErrors, 0.99), 0.005, "E2")
        XCTAssertLessThan(allErrors.max() ?? .infinity, 0.020,
                          "X1: a >20 mm residual is a different branch, not FK noise")
    }

    // MARK: - X3: is it better than the code it replaced

    /// The A/B on THIS code: the same poses solved with every `WrapEllipsoid`
    /// active and then deactivated, against OpenSim's own derivative.
    ///
    /// Deactivating is the right "before". The alternative — OpenSim's wrap-off
    /// column — carries every other difference between the two implementations
    /// as well (Nimble's FK, live `MovingPathPoint` evaluation and the
    /// finite-difference step), and on these muscles those are millimetres.
    /// Subtracting two things that differ in two ways attributes nothing.
    func testTurningTheEllipsoidsOnMovesTheMomentArmsTowardsOpenSim() {
        let pairs = WrapValidationHarness.ablation.filter { $0.centralDifference != nil }
        guard !pairs.isEmpty else { return XCTFail("the ablation collected nothing") }
        let moved = pairs.filter { $0.withEllipsoids != $0.withoutEllipsoids }
        let withOn = pairs.map { abs($0.withEllipsoids - ($0.centralDifference ?? 0)) }
        let withOff = pairs.map { abs($0.withoutEllipsoids - ($0.centralDifference ?? 0)) }
        let movedOn = moved.map { abs($0.withEllipsoids - ($0.centralDifference ?? 0)) }
        let movedOff = moved.map { abs($0.withoutEllipsoids - ($0.centralDifference ?? 0)) }
        print("ABLATION pairs=\(pairs.count) changed_by_the_ellipsoid=\(moved.count)")
        print(WrapValidationHarness.describe(withOn, label: "ABLATION all: ellipsoids ON"))
        print(WrapValidationHarness.describe(withOff, label: "ABLATION all: ellipsoids OFF"))
        print(WrapValidationHarness.describe(movedOn, label: "ABLATION changed: ellipsoids ON"))
        print(WrapValidationHarness.describe(movedOff, label: "ABLATION changed: ellipsoids OFF"))
        for sample in moved.sorted(by: {
            abs($0.withoutEllipsoids - ($0.centralDifference ?? 0))
                > abs($1.withoutEllipsoids - ($1.centralDifference ?? 0))
        }).prefix(8) {
            print(String(format: "  ABLATION %@ %@ %@  on %+.5f  off %+.5f  ref %+.5f",
                         sample.pose, sample.muscle, sample.coordinate,
                         sample.withEllipsoids, sample.withoutEllipsoids,
                         sample.centralDifference ?? .nan))
        }

        XCTAssertGreaterThan(moved.count, 0,
                             "X5: no pair changed when the ellipsoids were deactivated, so "
                             + "the solver never ran")
        let improved = moved.filter {
            abs($0.withEllipsoids - ($0.centralDifference ?? 0))
                < abs($0.withoutEllipsoids - ($0.centralDifference ?? 0))
        }
        print("ABLATION improved=\(improved.count) of \(moved.count) changed pairs "
              + "(\(String(format: "%.1f", 100.0 * Double(improved.count) / Double(max(1, moved.count))))%)")
        XCTAssertLessThan(WrapValidationHarness.percentile(movedOn, 0.5),
                          WrapValidationHarness.percentile(movedOff, 0.5),
                          "X3: on the pairs the ellipsoid changes, it must move them TOWARDS "
                          + "OpenSim. Wrapping that does not help is wrapping that is wrong")
        XCTAssertLessThan(movedOn.max() ?? .infinity, movedOff.max() ?? 0,
                          "X3: and the worst case must improve too")
    }

    /// What is LEFT after the solver lands, and did the solver put it there?
    ///
    /// The same pairs ranked by the residual that SURVIVES, each printed beside
    /// its ellipsoids-off value. A residual the ellipsoid caused has a much
    /// smaller "off"; another cross-implementation residual can have an "off"
    /// of the same size. This report deliberately does not pre-attribute the
    /// remainder: FullBody's four MovingPathPoints now use exact SimmSpline
    /// evaluation, and the measured 2.679 mm maximum must stand on its own.
    func testTheResidualThatSurvivesIsReportedWithoutPreAttribution() {
        let pairs = WrapValidationHarness.ablation.filter { $0.centralDifference != nil }
        guard !pairs.isEmpty else { return XCTFail("the ablation collected nothing") }
        let ranked = pairs.sorted {
            abs($0.withEllipsoids - ($0.centralDifference ?? 0))
                > abs($1.withEllipsoids - ($1.centralDifference ?? 0))
        }
        for sample in ranked.prefix(10) {
            let on = abs(sample.withEllipsoids - (sample.centralDifference ?? 0))
            let off = abs(sample.withoutEllipsoids - (sample.centralDifference ?? 0))
            print(String(format: "SURVIVING %@ %@ %@  |on-ref| %.6f  |off-ref| %.6f  "
                         + "ellipsoid_changed=%@",
                         sample.pose, sample.muscle, sample.coordinate, on, off,
                         sample.withEllipsoids == sample.withoutEllipsoids ? "no" : "yes"))
        }
        // Nothing is gated here that is not gated above; this exists so the
        // number that survives is on the record beside the A/B observation.
        XCTAssertGreaterThan(ranked.count, 10)
    }

    /// The same question against the column a reader of OpenSim would quote.
    /// Reported and gated only on the SIGN, because on this population the
    /// analytic-versus-central-difference gap (amendment A1) and other
    /// cross-implementation residuals are reported independently of the wrap's
    /// own effect.
    func testEllipsoidWrappingAgainstTheAnalyticColumnIsReported() {
        let pairs = Self.samples
        let errors = pairs.map { abs($0.ours - $0.wrapOn) }
        let straightLine = pairs.map { abs($0.wrapOff - $0.wrapOn) }
        print(WrapValidationHarness.describe(errors, label: "ELLIPSOID: ours vs analytic reference"))
        print(WrapValidationHarness.describe(straightLine,
                                             label: "ELLIPSOID: straight line vs analytic reference"))
        print(WrapValidationHarness.worstOffenders(in: pairs, by: { abs($0.ours - $0.wrapOn) },
                                                   label: "largest residuals vs the ANALYTIC column"))
        XCTAssertGreaterThan(straightLine.max() ?? 0, 0.001,
                             "the straight line already matched the reference on these "
                             + "muscles, so there was nothing here to fix and this suite "
                             + "is measuring nothing")
    }

    // MARK: - E3 / X2: the backwards-quadrant detector

    /// All eight ellipsoid wrap objects carry a real `<quadrant>` (`x`, `-x`,
    /// `-y`, `z`), so the mirror-to-the-active-side branch runs, and getting it
    /// backwards produces a perfectly plausible path on the wrong side of the
    /// humerus. The primary check compares the ellipsoid's causal A/B effect;
    /// the whole-sweep total sign keeps the original fixed 1 mm tripwire.
    func testEllipsoidMusclesNeverPointTheWrongWay() {
        let effectResolution = 0.001
        let singleWrapAblation = WrapValidationHarness.ablation.filter { $0.wrapCount == 1 }
        let nonFiniteEffects = singleWrapAblation.filter {
            !$0.withEllipsoids.isFinite || !$0.withoutEllipsoids.isFinite
                || !$0.wrapOn.isFinite || !$0.wrapOff.isFinite
        }
        XCTAssertEqual(nonFiniteEffects.count, 0,
                       "non-finite A/B data cannot satisfy a direction gate")
        let effects = singleWrapAblation.filter {
            abs($0.wrapOn - $0.wrapOff) >= effectResolution
        }
        let effectFlips = effects.filter {
            let actual = $0.withEllipsoids - $0.withoutEllipsoids
            let reference = $0.wrapOn - $0.wrapOff
            return actual * reference <= 0
        }
        print("SIGN-EFFECT resolution_mm=1 pairs=\(effects.count) flipped=\(effectFlips.count)")
        for sample in effectFlips.prefix(10) {
            print(String(format: "  EFFECT-FLIP %@ %@ %@ actual %+.5f reference %+.5f",
                         sample.pose, sample.muscle, sample.coordinate,
                         sample.withEllipsoids - sample.withoutEllipsoids,
                         sample.wrapOn - sample.wrapOff))
        }
        XCTAssertGreaterThan(effects.count, 0, "the ellipsoid A/B has no resolved wrap effect")
        XCTAssertEqual(effectFlips.count, 0,
                       "the ellipsoid must move the path in OpenSim's wrap-effect direction")

        let totalResolution = 0.001  // original fixed E3/X2 sign tripwire
        let single = Self.samples.filter {
            $0.wrapCount == 1 && abs($0.centralDifference ?? 0) >= totalResolution
        }
        let flipped = single.filter { ($0.ours < 0) != (($0.centralDifference ?? 0) < 0) }
        let flippedMulti = Self.samples.filter {
            $0.wrapCount > 1 && ($0.ours < 0) != (($0.centralDifference ?? 0) < 0)
                && abs($0.centralDifference ?? 0) >= totalResolution
        }
        let oneToFive = Self.samples.filter {
            $0.wrapCount == 1
                && abs($0.centralDifference ?? 0) >= totalResolution
                && abs($0.centralDifference ?? 0) < 0.005
                && ($0.ours < 0) != (($0.centralDifference ?? 0) < 0)
        }
        let analytic = Self.samples.filter { abs($0.wrapOn) >= 0.001 }
        let flippedAnalytic = analytic.filter { ($0.ours < 0) != ($0.wrapOn < 0) }
        let flippedBefore = analytic.filter { ($0.wrapOff < 0) != ($0.wrapOn < 0) }
        print("SIGN-ELLIPSOID single=\(single.count) flipped=\(flipped.count) | "
              + "one-to-five-mm=\(oneToFive.count) | multi flipped=\(flippedMulti.count) | "
              + "vs ANALYTIC: pairs=\(analytic.count) "
              + "flipped_now=\(flippedAnalytic.count) "
              + "flipped_by_the_straight_line=\(flippedBefore.count)")
        for sample in (flipped + flippedMulti + flippedAnalytic).prefix(10) {
            print(String(format: "  SIGN-FLIP %@ %@ %@ ours %+.5f central %+.5f analytic %+.5f",
                         sample.pose, sample.muscle, sample.coordinate, sample.ours,
                         sample.centralDifference ?? .nan, sample.wrapOn))
        }
        XCTAssertGreaterThan(single.count, 0, "no single-wrap sample cleared E3's 1 mm contract")
        XCTAssertEqual(flipped.count, 0, "E3/X2: a backwards quadrant looks exactly like this")
        XCTAssertGreaterThan(flippedBefore.count, 0,
                             "the straight line flipped no sign on these muscles, so this "
                             + "test cannot tell a fixed one from an untouched one")
        XCTAssertLessThan(flippedAnalytic.count, flippedBefore.count / 10,
                          "the straight line flipped \(flippedBefore.count) signs against "
                          + "the analytic column; wrapping must remove almost all of them")
    }

    // MARK: - E4 / E5: length and engagement

    func testEllipsoidMusclePathLengthsMatchTheReference() {
        let single = Self.lengthSamples.filter { $0.wrapCount == 1 }
        let multi = Self.lengthSamples.filter { $0.wrapCount > 1 }
        let singleErrors = single.map { abs($0.ours - $0.wrapOn) }
        let multiErrors = multi.map { abs($0.ours - $0.wrapOn) }
        print(WrapValidationHarness.describe(singleErrors,
                                             label: "ELLIPSOID SINGLE-WRAP: path length"))
        print(WrapValidationHarness.describe(multiErrors,
                                             label: "ELLIPSOID MULTI-WRAP: path length"))
        for sample in multi.sorted(by: { abs($0.ours - $0.wrapOn) > abs($1.ours - $1.wrapOn) })
                           .prefix(4) {
            print(String(format: "  LENGTH-MULTI %@ %@ ours %.6f ref %.6f",
                         sample.pose, sample.muscle, sample.ours, sample.wrapOn))
        }
        XCTAssertGreaterThan(single.count, 0)
        XCTAssertGreaterThan(multi.count, 0)
        XCTAssertLessThan(singleErrors.max() ?? .infinity, 0.005, "E4")
        XCTAssertLessThan(multiErrors.max() ?? .infinity, 0.020,
                          "X1's envelope still applies to the muscles OpenSim solves "
                          + "iteratively")
    }

    /// The ellipsoid engages ONLY when the straight segment actually pierces it,
    /// so engagement is a sharp yes/no that a length cannot match by accident.
    func testEllipsoidWrapEngagementAgreesWithOpenSim() {
        let rows = Self.lengthSamples
        let agree = rows.filter { $0.ourWrapPoints == $0.referenceWrapPoints }
        let engagedInReference = rows.filter { $0.referenceWrapPoints > 0 }
        let engagedHere = rows.filter { $0.ourWrapPoints > 0 }
        let rate = Double(agree.count) / Double(max(1, rows.count))
        print(String(format: "ENGAGEMENT-ELLIPSOID rows=%d agree=%d (%.2f%%) "
                     + "engaged_ref=%d engaged_ours=%d",
                     rows.count, agree.count, rate * 100,
                     engagedInReference.count, engagedHere.count))
        for sample in rows.filter({ $0.ourWrapPoints != $0.referenceWrapPoints }).prefix(12) {
            print("  ENGAGEMENT-DIFF \(sample.pose) \(sample.muscle) "
                  + "ours=\(sample.ourWrapPoints) ref=\(sample.referenceWrapPoints)")
        }
        XCTAssertGreaterThan(engagedInReference.count, 0,
                             "the reference never engages an ellipsoid at these poses, so "
                             + "this suite would pass with no solver at all")
        XCTAssertGreaterThanOrEqual(rate, 0.99, "E5")
    }

    // MARK: - E8 / X5: the cost

    /// The named risk, as a PAIRED difference rather than two runs subtracted:
    /// the same process, the same poses, the same solver, with every
    /// `WrapEllipsoid` active and then deactivated.
    ///
    /// The fan technique inside `WrapEllipsoid::wrapLine` is ~300
    /// point-to-ellipsoid Newton solves, and a standalone `-O2` benchmark puts
    /// one engaged ellipsoid solve at 12–41 µs against 0.09 µs for a one-wrap
    /// cylinder. What keeps that affordable is that the ellipsoid only reaches
    /// the fan when the segment actually pierces it; every other case is an
    /// early-out.
    func testEllipsoidCostAgainstTheCylinderOnlyChain() {
        let ab = WrapValidationHarness.costAB
        guard !ab.poses.isEmpty else { return XCTFail("the cost A/B collected nothing") }
        let on = ab.withEllipsoids.reduce(0, +) / Double(ab.withEllipsoids.count)
        let off = ab.withoutEllipsoids.reduce(0, +) / Double(ab.withoutEllipsoids.count)
        for (index, pose) in ab.poses.enumerated() {
            print(String(format: "ELLIPSOID-COST %@  with %.1f ms  without %.1f ms  delta %+.1f ms",
                         pose, ab.withEllipsoids[index], ab.withoutEllipsoids[index],
                         ab.withEllipsoids[index] - ab.withoutEllipsoids[index]))
        }
        print(String(format: "ELLIPSOID-COST n=%d mean with %.1f ms  without %.1f ms  "
                     + "ratio %.2fx  (Debug, iOS Simulator, 169 coordinates x 520 muscles)",
                     ab.poses.count, on, off, on / max(off, 1e-9)))
        XCTAssertGreaterThan(on, off,
                             "X5: deactivating every WrapEllipsoid changed nothing, so the "
                             + "ellipsoid solver never ran and every gate above is about "
                             + "the cylinders")
        XCTAssertLessThan(on / max(off, 1e-9), 3.0,
                          "E8: the ellipsoid must not triple the moment-arm solve. Beyond "
                          + "that the honest outcome is to ship the cylinders and leave the "
                          + "8 ellipsoids counted and disclosed")
    }
}
