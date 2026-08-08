import XCTest
@testable import BioMotion

/// The fixture that gives this project an authoritative moment-arm reference
/// for the first time, and the measurement of how far the shipped straight-line
/// path is from it.
///
/// Read `OpenSimReferenceFixture` first: `...WrapOn` is OpenSim solving all 76
/// `PathWrap`s, `...WrapOff` is the same model with every `WrapObject`
/// deactivated, which is the shortcut `MomentArmComputer` takes today.
final class OpenSimReferenceTests: XCTestCase {

    private static var cached: OpenSimReferenceFixture.Table?

    private func table() throws -> OpenSimReferenceFixture.Table {
        if let cached = Self.cached { return cached }
        let loaded = try OpenSimReferenceFixture.load(bundle: Bundle(for: type(of: self)))
        Self.cached = loaded
        return loaded
    }

    // MARK: - The fixture itself

    func testFixtureLoadsAndIsWellFormed() throws {
        let table = try table()
        XCTAssertEqual(table.coordinateNames.count, 169,
                       "FullBody.osim has 169 coordinates")
        XCTAssertEqual(table.poses.count, 173)
        XCTAssertEqual(table.muscles.count, 104,
                       "66 muscles carry a PathWrap; the rest are the muscles the "
                       + "product names, which the QP redistributes load onto")
        XCTAssertEqual(table.muscles.filter(\.carriesPathWrap).count, 66)
        XCTAssertEqual(table.rows.count, table.poses.count * table.muscles.count)

        for pose in table.poses {
            XCTAssertEqual(pose.values.count, table.coordinateNames.count)
            XCTAssertTrue(pose.values.allSatisfy { $0.isFinite })
        }
        for row in table.rows {
            let arity = table.muscles[row.muscleIndex].coordinates.count
            XCTAssertEqual(row.momentArmsWrapOn.count, arity)
            XCTAssertEqual(row.momentArmsWrapOff.count, arity)
            XCTAssertTrue(row.lengthWrapOn.isFinite && row.lengthWrapOn > 0)
            XCTAssertTrue(row.lengthWrapOff.isFinite && row.lengthWrapOff > 0)
            XCTAssertGreaterThanOrEqual(row.wrapPoints, 0)
        }
    }

    /// A malformed fixture must produce a named error, never a trap. The
    /// previous generated fixture in this target killed the whole test host on
    /// its first data line.
    func testMalformedFixtureThrowsInsteadOfTrapping() {
        let cases: [(String, String)] = [
            ("format wrong-id\n", "bad format id"),
            ("format \(OpenSimReferenceFixture.formatId)\ncoordinates a b\nposes 1\nmuscles 1\n"
             + "pose p nan 0.0\n", "nan is not a decimal"),
            ("format \(OpenSimReferenceFixture.formatId)\ncoordinates a b\nposes 1\nmuscles 1\n"
             + "pose p 1e3 0.0\n", "exponent is not a plain decimal"),
            ("format \(OpenSimReferenceFixture.formatId)\ncoordinates a b\nposes 1\nmuscles 1\n"
             + "pose p 0.0 0.0\nmuscle m 1 1 a\nrow 0 0 0 1.0 1.0 0.1 0.1\n"
             + "row 0 0 0 1.0 1.0 0.1 0.1\n",
             "a repeated row is out of order"),
            ("format \(OpenSimReferenceFixture.formatId)\ncoordinates a b\nposes 1\nmuscles 1\n"
             + "pose p 0.0 0.0\nmuscle m 1 1 zzz\n", "unknown coordinate name"),
        ]
        for (text, why) in cases {
            XCTAssertThrowsError(try OpenSimReferenceFixture.parse(text), why)
        }
    }

    /// Every muscle the product NAMES on screen has to be in the reference, or
    /// the reference cannot speak to the claim that was retired.
    func testEveryNamedMuscleTheModelHasIsCovered() throws {
        let table = try table()
        let covered = Set(table.muscles.map(\.name))
        let bases = Set(covered.map { name -> String in
            (name.hasSuffix("_r") || name.hasSuffix("_l")) ? String(name.dropLast(2)) : name
        })
        // Not every display name exists in FullBody.osim -- the table is shared
        // with Rajagopal2016, which has `bflh` where FullBody has `bflh140`.
        // Assert on the ones the model actually carries.
        let missing = GaitLoadSummary.displayNames.keys.filter { base in
            !bases.contains(base) && MuscleNameProbe.fullBodyHasBase(base)
        }
        XCTAssertEqual(missing.sorted(), [],
                       "display-name muscles absent from the OpenSim reference")
    }

    /// `wrapPoints == 0` has to MEAN something, or the flag cannot be used to
    /// detect the engage/disengage discontinuity. Where OpenSim inserted no
    /// wrap point the two models must agree on the length exactly.
    func testWrapPointFlagSeparatesAgreementFromDisagreement() throws {
        let table = try table()
        var agreeingWhenDisengaged = 0
        var disengaged = 0
        var engaged = 0
        var worstDisengagedDifference = 0.0
        for row in table.rows {
            let difference = abs(row.lengthWrapOn - row.lengthWrapOff)
            if row.wrapPoints == 0 {
                disengaged += 1
                worstDisengagedDifference = max(worstDisengagedDifference, difference)
                if difference < 1e-9 { agreeingWhenDisengaged += 1 }
            } else {
                engaged += 1
            }
        }
        XCTAssertGreaterThan(engaged, 0, "no row has wrapping engaged")
        XCTAssertGreaterThan(disengaged, 0, "no row has wrapping disengaged")
        XCTAssertEqual(agreeingWhenDisengaged, disengaged,
                       "a row with no wrap point must have identical lengths; "
                       + "worst disagreement \(worstDisengagedDifference) m")
    }

    /// The risk named before any code was written: `dL/dq` is discontinuous
    /// where a muscle starts or stops wrapping, and a centred difference
    /// straddling that switch invents a moment arm. The fixture has to CONTAIN
    /// such switches or the next stage cannot test for them.
    func testSweepsBracketWrapEngagementSwitches() throws {
        let table = try table()
        var switches = 0
        var musclesThatSwitch = Set<String>()
        for prefix in ["knee_sweep_", "hip_sweep_", "ankle_sweep_",
                       "elbow_sweep_", "shoulder_sweep_"] {
            let indices = table.poses.enumerated()
                .filter { $0.element.id.hasPrefix(prefix) }
                .map(\.offset)
            for muscle in 0..<table.muscles.count {
                for (a, b) in zip(indices, indices.dropFirst()) {
                    guard let first = table.row(pose: a, muscle: muscle),
                          let second = table.row(pose: b, muscle: muscle) else { continue }
                    if (first.wrapPoints == 0) != (second.wrapPoints == 0) {
                        switches += 1
                        musclesThatSwitch.insert(table.muscles[muscle].name)
                    }
                }
            }
        }
        // 25 transitions on 23 DISTINCT muscles -- two of them switch in more
        // than one sweep, which is why the per-sweep tallies sum to 25 muscles
        // and this set holds 23.
        XCTAssertEqual(switches, 25,
                       "the sweeps must bracket wrap engage/disengage transitions, or "
                       + "nothing in this fixture can test the discontinuity")
        XCTAssertEqual(musclesThatSwitch.count, 23)
    }

    // MARK: - The defect, counted

    /// The size of the thing the wrap solver has to fix, computed from the
    /// shipped fixture so the numbers in STATUS.md cannot drift away from it.
    ///
    /// Every count is over (pose, muscle, coordinate) triples. Pairs whose
    /// REFERENCE moment arm is under 1 mm are excluded from every ratio and
    /// counted separately -- a ratio there is a statement about its own
    /// denominator.
    func testStraightLineDefectIsWhatStatusRecords() throws {
        let table = try table()

        struct Tally {
            var total = 0, tiny = 0, signFlips = 0, over10 = 0, over100 = 0
            var worst = 0.0
        }
        func tally(_ include: (OpenSimReferenceFixture.Muscle) -> Bool) -> Tally {
            var out = Tally()
            for row in table.rows {
                let muscle = table.muscles[row.muscleIndex]
                guard include(muscle) else { continue }
                for (reference, straightLine) in zip(row.momentArmsWrapOn,
                                                     row.momentArmsWrapOff) {
                    out.total += 1
                    guard abs(reference) >= 1e-3 else { out.tiny += 1; continue }
                    if reference * straightLine < 0 { out.signFlips += 1 }
                    let relative = abs(reference - straightLine) / abs(reference)
                    if relative > 0.10 { out.over10 += 1 }
                    if relative > 1.00 { out.over100 += 1 }
                    out.worst = max(out.worst, relative)
                }
            }
            return out
        }

        let wrapped = tally { $0.carriesPathWrap }
        XCTAssertEqual(wrapped.total, 41866)
        XCTAssertEqual(wrapped.tiny, 2899)
        XCTAssertEqual(wrapped.signFlips, 3769,
                       "9.0% of the pairs on wrapped muscles point the WRONG WAY. A "
                       + "sign-flipped moment arm is not rescaled by the QP: it pins "
                       + "the muscle to aMin on both sides and reads exactly 0.0% "
                       + "left/right (STATUS, `isSaturated` entry)")
        XCTAssertEqual(wrapped.over10, 20707)
        XCTAssertEqual(wrapped.over100, 7932)
        XCTAssertEqual(wrapped.worst, 6.9472, accuracy: 0.0005)

        let named = tally { muscle in
            let base = muscle.name.hasSuffix("_r") || muscle.name.hasSuffix("_l")
                ? String(muscle.name.dropLast(2)) : muscle.name
            return GaitLoadSummary.displayNames[base] != nil
        }
        XCTAssertEqual(named.total, 37714)
        XCTAssertEqual(named.tiny, 1930)
        XCTAssertEqual(named.signFlips, 232)
        XCTAssertEqual(named.over10, 5811)
        XCTAssertEqual(named.over100, 241)
        XCTAssertEqual(named.worst, 3.7688, accuracy: 0.0005)

        // The control that says the two columns differ by WRAPPING and nothing
        // else. Every muscle in this fixture that carries no PathWrap has to
        // agree to the last stored digit at every pose.
        var unwrappedDisagreements = 0
        var worstUnwrapped = 0.0
        for row in table.rows where !table.muscles[row.muscleIndex].carriesPathWrap {
            for (reference, straightLine) in zip(row.momentArmsWrapOn,
                                                 row.momentArmsWrapOff) {
                let difference = abs(reference - straightLine)
                worstUnwrapped = max(worstUnwrapped, difference)
                if difference > 0 { unwrappedDisagreements += 1 }
            }
            if abs(row.lengthWrapOn - row.lengthWrapOff) > 0 { unwrappedDisagreements += 1 }
        }
        XCTAssertEqual(unwrappedDisagreements, 0,
                       "a muscle with no PathWrap must be identical in both models; "
                       + "worst \(worstUnwrapped) m")
    }
}

/// Tiny helper so the coverage test can ask the model a question without
/// hard-coding a second copy of its muscle list.
enum MuscleNameProbe {
    private static var bases: Set<String> = []

    static func fullBodyHasBase(_ base: String) -> Bool {
        if bases.isEmpty { load() }
        return bases.contains(base)
    }

    private static func load() {
        let bundle = Bundle(for: OpenSimReferenceTests.self)
        guard let path = bundle.path(forResource: "FullBody", ofType: "osim"),
              let text = try? String(contentsOfFile: path, encoding: .utf8) else { return }
        // `<Millard2012EquilibriumMuscle name="recfem_r">` and friends.
        var found = Set<String>()
        var search = text[text.startIndex...]
        while let range = search.range(of: "Muscle name=\"") {
            let rest = search[range.upperBound...]
            if let close = rest.firstIndex(of: "\"") {
                let name = String(rest[rest.startIndex..<close])
                found.insert(name.hasSuffix("_r") || name.hasSuffix("_l")
                             ? String(name.dropLast(2)) : name)
            }
            search = search[range.upperBound...]
        }
        bases = found
    }
}
