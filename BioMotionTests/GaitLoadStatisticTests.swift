import XCTest
import UIKit
@testable import BioMotion

/// **The left/right statistic, and the count-dependence it must not have.**
///
/// The product's headline finding is "this muscle is N % harder on the left".
/// Until this file existed that number was a MAX over each side's usable stance
/// frames, and the two sides do not contribute the same number of frames — the
/// inequality is produced by the very asymmetry being measured. With `taps = 5`
/// a contact of `n` samples yields exactly `n − 4` frames with a clean derivative
/// window, so one extra sample of contact DOUBLES a leg's frame count.
///
/// `E[max of n]` grows with `n`. So the side that happened to contribute more
/// frames read higher for a purely statistical reason, on top of (and able to
/// flip the sign of) the real effect.
///
/// These tests measure that bias on the shipping code path, pin the replacement
/// statistic's definition exactly, and check the properties the replacement is
/// chosen for.
final class GaitLoadStatisticTests: XCTestCase {

    // MARK: - The definition, by hand

    /// **One sample per contact — the middle of its usable frames — averaged
    /// over that side's contacts.** Hand-computed, so the definition is pinned
    /// by arithmetic and not by whatever the code does.
    func testTheStatisticIsTheMeanOverContactsOfEachContactsMiddleSample() throws {
        let report = try Self.usableReport(bundle: bundle)
        // Left contact A holds three usable frames: the middle one is the
        // sample. Left contact B holds two: the two middle ones are averaged,
        // the same convention `median(_:)` uses.
        // Right contact C holds one.
        let frames =
            Self.contact(side: -1, firstID: 0, values: [0.40, 0.80, 0.60])
            + Self.contact(side: 1, firstID: 10, values: [0.50])
            + Self.contact(side: -1, firstID: 20, values: [0.20, 0.30])
            + Self.contact(side: 1, firstID: 30, values: [0.50])

        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        let load = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
        // A → 0.80 (middle of three), B → 0.25 (mean of the two middle).
        XCTAssertEqual(load.leftLoad, 0.5 * (0.80 + 0.25), accuracy: 1e-12)
        XCTAssertEqual(load.rightLoad, 0.50, accuracy: 1e-12)
        XCTAssertEqual(load.leftContacts, 2)
        XCTAssertEqual(load.rightContacts, 2)
        XCTAssertEqual(s.leftContactCount, 2)
        XCTAssertEqual(s.rightContactCount, 2)
        // And it is NOT the max: the old rule would have published 0.80 vs 0.50,
        // a 46 % left-high claim built out of one frame.
        XCTAssertEqual(Self.oldMaxStatistic(frames).left, 0.80, accuracy: 1e-12)
        XCTAssertEqual(load.differencePercent, 100 * (0.525 - 0.50) / 0.5125, accuracy: 1e-9)
    }

    /// A contact is a maximal run of consecutive stance frames on ONE side.
    /// Flight — or any frame the plan does not call stance — ends it, and so
    /// does the other foot landing.
    func testAContactEndsAtFlightAndAtTheOtherFoot() throws {
        let report = try Self.usableReport(bundle: bundle)
        // Two left contacts separated by a flight frame: 0.20 and 0.60. If the
        // flight frame did not split them, the statistic would take the middle
        // of one four-frame run (0.20, 0.20, 0.60, 0.60) → 0.40, which is the
        // same number by symmetry, so the values are chosen to differ: one run
        // of [0.10, 0.20, 0.90] vs two runs of [0.10] and [0.20, 0.90].
        let split = Self.contact(side: -1, firstID: 0, values: [0.10])
            + [Self.flight(id: 5)]
            + Self.contact(side: -1, firstID: 10, values: [0.20, 0.90])
            + Self.contact(side: 1, firstID: 20, values: [0.50])
            + Self.contact(side: 1, firstID: 30, values: [0.50])
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: split, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        let load = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
        // Two left contacts: 0.10, and mean(0.20, 0.90) = 0.55 → 0.325.
        XCTAssertEqual(load.leftLoad, 0.5 * (0.10 + 0.55), accuracy: 1e-12)
        XCTAssertEqual(load.leftContacts, 2)
        // The two right runs are adjacent in time but on the same side with no
        // frame between them, so they ARE one contact — the array is the only
        // record of the boundary and it says they are contiguous.
        XCTAssertEqual(load.rightContacts, 1)
        XCTAssertEqual(s.rightContactCount, 1)
    }

    // MARK: - The bias, measured

    /// **The finding, reproduced and then removed, on the shipping path.**
    ///
    /// A symmetric runner — both legs drawn from the SAME activation
    /// distribution — filmed so that the left leg's contacts hold 6 usable
    /// frames and the right's hold 2. Nothing about this runner is asymmetric;
    /// only the sample counts are.
    ///
    /// Because a single clip is one realisation, the quantity of interest is the
    /// EXPECTATION, so this runs 400 seeded trials and averages. The old
    /// statistic's mean is a bias; the new one's is zero to within the standard
    /// error of the mean.
    func testAMaxOverUnequalFrameCountsFabricatesAsymmetryAndTheMeanOverContactsDoesNot() throws {
        let report = try Self.usableReport(bundle: bundle)
        let trials = 400
        var oldPercents: [Double] = []
        var newPercents: [Double] = []

        for seed in 0..<trials {
            var rng = Deterministic(seed: UInt64(seed) &+ 1)
            var frames: [OfflineResultStore.FrameResult] = []
            var id = 0
            for _ in 0..<6 {
                // Same mean, same scatter, same distribution on both sides.
                let leftValues = (0..<6).map { _ in rng.activation() }
                let rightValues = (0..<2).map { _ in rng.activation() }
                frames += Self.contact(side: -1, firstID: id, values: leftValues); id += 10
                frames += Self.contact(side: 1, firstID: id, values: rightValues); id += 10
            }
            let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                       framesPerSecond: 30, filterTaps: 5))
            let load = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
            XCTAssertEqual(load.leftContacts, 6)
            XCTAssertEqual(load.rightContacts, 6)
            newPercents.append(load.differencePercent)

            let old = Self.oldMaxStatistic(frames)
            oldPercents.append(200 * (old.left - old.right) / (old.left + old.right))
        }

        let oldMean = oldPercents.reduce(0, +) / Double(trials)
        let newMean = newPercents.reduce(0, +) / Double(trials)
        let newStandardError = Self.standardError(newPercents)
        let oldPositive = oldPercents.filter { $0 > 0 }.count
        let newPositive = newPercents.filter { $0 > 0 }.count
        print("GAIT-METRIC load_statistic_bias trials=\(trials) "
              + "old_max_mean_percent=\(oldMean) old_positive=\(oldPositive) "
              + "new_contact_mean_percent=\(newMean) new_positive=\(newPositive) "
              + "new_standard_error=\(newStandardError)")

        // The old rule: a systematic left-high reading from a symmetric runner.
        // MEASURED at this scatter (σ = 0.12 of activation, the shipped 5-tap
        // window's own amplification territory): +8.07 % on average and
        // left-high on 297 of 400 clips.
        XCTAssertGreaterThan(oldMean, 5.0,
                             "max over 36 frames against max over 12 must read left-high")
        XCTAssertGreaterThan(oldPositive, trials * 65 / 100,
                             "and on most individual clips, not on average only")

        // The new rule: unbiased. `mean` has expectation equal to the population
        // mean for ANY distribution and ANY sample count, which is the whole
        // reason it was chosen over a maximum.
        XCTAssertLessThan(abs(newMean), 3 * newStandardError,
                          "the new statistic's mean must be zero to within its own standard error")
        XCTAssertLessThan(abs(newMean), 1.0, "and small in absolute terms: \(newMean) %")
        XCTAssertGreaterThan(newPositive, trials * 2 / 5, "sign must be a coin flip, not a bias")
        XCTAssertLessThan(newPositive, trials * 3 / 5)

        // And the size of what was removed, against the gate that would have let
        // it through. `video_012` publishes ±10.1 %; the bias measures 8.1 %,
        // i.e. 80 % of the publication floor. It is not on its own publishable —
        // it does not have to be. It rides on top of the real effect, so a true
        // 3 % difference reads 11 % and gets published as a finding, and a true
        // 8 % right-high difference reads even and is refused.
        XCTAssertGreaterThan(oldMean, 0.5 * report.resolution.resolvableAsymmetryPercent,
                             "the fabricated bias is a large fraction of the publication floor")
        print("GAIT-METRIC load_statistic_bias_vs_resolution old_mean=\(oldMean) "
              + "resolution=\(report.resolution.resolvableAsymmetryPercent)")
    }

    /// The same measurement with the frame counts EQUAL: the old rule was not
    /// wrong in general, it was wrong when the counts differed. Without this
    /// control the test above would also pass if the new statistic simply read
    /// zero always.
    func testWithEqualFrameCountsNeitherStatisticIsBiasedAndBothSeeARealDifference() throws {
        let report = try Self.usableReport(bundle: bundle)
        var oldPercents: [Double] = []
        var newPercents: [Double] = []
        for seed in 0..<200 {
            var rng = Deterministic(seed: UInt64(seed) &+ 5000)
            var frames: [OfflineResultStore.FrameResult] = []
            var id = 0
            for _ in 0..<6 {
                let l = (0..<4).map { _ in rng.activation() }
                let r = (0..<4).map { _ in rng.activation() }
                frames += Self.contact(side: -1, firstID: id, values: l); id += 10
                frames += Self.contact(side: 1, firstID: id, values: r); id += 10
            }
            let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                       framesPerSecond: 30, filterTaps: 5))
            newPercents.append(try XCTUnwrap(s.ranked.first { $0.id == "glmax1" }).differencePercent)
            let old = Self.oldMaxStatistic(frames)
            oldPercents.append(200 * (old.left - old.right) / (old.left + old.right))
        }
        let oldMean = oldPercents.reduce(0, +) / 200
        let newMean = newPercents.reduce(0, +) / 200
        print("GAIT-METRIC load_statistic_equal_counts old_mean_percent=\(oldMean) "
              + "new_mean_percent=\(newMean)")
        XCTAssertLessThan(abs(oldMean), 1.5, "equal counts, no max bias")
        XCTAssertLessThan(abs(newMean), 1.5)

        // And a genuinely stronger left leg is still reported as one — the new
        // statistic must not be a way of always answering "even".
        let stronger = Self.contact(side: -1, firstID: 0, values: [0.80, 0.80])
            + Self.contact(side: 1, firstID: 10, values: [0.50, 0.50])
            + Self.contact(side: -1, firstID: 20, values: [0.80, 0.80])
            + Self.contact(side: 1, firstID: 30, values: [0.50, 0.50])
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: stronger, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        let load = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
        XCTAssertEqual(load.differencePercent, 100 * 0.30 / 0.65, accuracy: 1e-9)
        XCTAssertTrue(s.permits(differencePercent: load.differencePercent))
        XCTAssertTrue(s.claim(for: load).contains("harder on the left"))
    }

    /// The residual count-dependence, stated rather than hidden: the middle
    /// sample of a contact sits at `sin(π·φ)` of the modelled peak — 1.000 for an
    /// odd number of usable frames, 0.966 for an even one. That parity is the
    /// whole remaining dependence on the frame count, it is bounded at 3.4 % of
    /// force scale, and unlike `E[max of n]` it does not grow.
    func testTheOnlyRemainingCountDependenceIsTheMidStanceParityAndItIsBounded() {
        /// The modelled half-sine force at the mid-most usable frame of a
        /// contact holding `usable` clean frames, as a fraction of that
        /// contact's peak. `taps = 5`, so a contact of `n` samples yields
        /// `n − 4` usable frames centred on mid-stance.
        func midStanceForceFraction(usable: Int) -> Double {
            let n = usable + 4
            let half = 2
            let ks = Array(half...(n - 1 - half))
            let m = ks.count
            let middle = m % 2 == 1 ? [ks[m / 2]] : [ks[m / 2 - 1], ks[m / 2]]
            return middle
                .map { sin(.pi * (Double($0) + 0.5) / Double(n)) }
                .reduce(0, +) / Double(middle.count)
        }
        var fractions: [Double] = []
        for usable in 1...9 { fractions.append(midStanceForceFraction(usable: usable)) }
        print("GAIT-METRIC mid_stance_force_fraction_by_usable_frames=\(fractions)")
        XCTAssertEqual(midStanceForceFraction(usable: 1), 1.0, accuracy: 1e-12, "n = 5")
        XCTAssertEqual(midStanceForceFraction(usable: 2), 0.96593, accuracy: 1e-4, "n = 6")
        XCTAssertEqual(midStanceForceFraction(usable: 3), 1.0, accuracy: 1e-12, "n = 7")
        let worst = 100 * (fractions.max()! - fractions.min()!)
            / (0.5 * (fractions.max()! + fractions.min()!))
        XCTAssertLessThan(worst, 3.5, "bounded at 3.4 % of force scale: measured \(worst) %")
        // It does not grow with the frame count — the last rung is no worse than
        // the first, which is the property `max` did not have.
        XCTAssertEqual(fractions[8], fractions[0], accuracy: 0.04)
    }

    // MARK: - Fixtures

    /// A seeded generator, so every number in this file is reproducible.
    struct Deterministic {
        private var state: UInt64
        init(seed: UInt64) { state = seed &* 6364136223846793005 &+ 1442695040888963407 }

        mutating func uniform() -> Double {
            state = state &* 6364136223846793005 &+ 1442695040888963407
            return Double(state >> 11) / Double(UInt64(1) << 53)
        }

        /// Normal(0.50, 0.12), clamped clear of the QP's saturation bound so
        /// `isSaturated` never enters the comparison.
        mutating func activation() -> Double {
            let u1 = Swift.max(uniform(), 1e-12), u2 = uniform()
            let z = (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
            return Swift.min(Swift.max(0.50 + 0.12 * z, 0.05), 0.95)
        }
    }

    /// **The statistic this file exists to replace**, computed independently
    /// here so the comparison is against the real old rule and not a
    /// description of it: the maximum over every usable stance frame of a side.
    static func oldMaxStatistic(_ frames: [OfflineResultStore.FrameResult])
        -> (left: Double, right: Double) {
        var l = 0.0, r = 0.0
        for frame in frames {
            guard frame.isGaitStance,
                  let outcome = frame.motionState.gaitOutcome,
                  outcome.isUsableForLoadComparison,
                  let muscle = frame.muscleResult else { continue }
            let onLeft = outcome.contactSide < 0
            let name = onLeft ? "glmax1_l" : "glmax1_r"
            guard let a = muscle.activations[name] else { continue }
            if onLeft { l = Swift.max(l, a) } else { r = Swift.max(r, a) }
        }
        return (l, r)
    }

    static func standardError(_ values: [Double]) -> Double {
        guard values.count > 1 else { return .infinity }
        let n = Double(values.count)
        let mean = values.reduce(0, +) / n
        let variance = values.map { ($0 - mean) * ($0 - mean) }.reduce(0, +) / (n - 1)
        return (variance / n).squareRoot()
    }

    /// One contact: consecutive stance frames on `side`, each carrying its own
    /// activation for both `glmax1_l` and `glmax1_r` (only the stance side's is
    /// ever credited, which is itself asserted elsewhere).
    static func contact(side: Int, firstID: Int, values: [Double])
        -> [OfflineResultStore.FrameResult] {
        values.enumerated().map { offset, value in
            stanceFrame(id: firstID + offset, side: side, activation: value)
        }
    }

    static func stanceFrame(id: Int, side: Int, activation: Double)
        -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.1,
            residualInBodyWeights: 0.1,
            contactSide: side,
            solverSawLeftContact: side < 0,
            solverSawRightContact: side > 0,
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: true)
        let muscle = NimbleEngine.MuscleOutput(
            activations: ["glmax1_l": activation, "glmax1_r": activation],
            forces: ["glmax1_l": activation * 1000, "glmax1_r": activation * 1000],
            converged: true,
            timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitStance, outcome: outcome))
    }

    static func flight(id: Int) -> OfflineResultStore.FrameResult {
        OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: nil,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitFlight, outcome: nil))
    }

    static func usableReport(bundle: Bundle) throws -> GaitReport {
        let frames = try GaitClipFixture.load("video_012", bundle: bundle).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        XCTAssertTrue(report.isUsable)
        return report
    }

    private var bundle: Bundle { Bundle(for: type(of: self)) }
}
