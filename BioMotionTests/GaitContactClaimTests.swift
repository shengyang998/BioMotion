import XCTest
@testable import BioMotion

/// **The gate on the one surviving product claim, and the Monte-Carlo that
/// sizes it.**
///
/// The running screen makes exactly one left/right statement — CONTACT TIME —
/// because it touches neither a moment arm nor the muscle QP. Until 2026-08-08
/// it published whenever `|contactAsymmetryPercent| >=
/// resolution.resolvableAsymmetryPercent`, and that floor is
/// `max(50/framesPerContact, max(stride-period CV, 100/stridePeriodFrames))`:
/// quantisation plus the STRIDE PERIOD's scatter. The statistic is the
/// difference of two MEANS OF CONTACT DURATIONS, whose scatter is a different
/// quantity — measured all along as `contactVariationPercent` and consumed by
/// nothing.
///
/// This is the same defect that killed the muscle claim one round earlier, on
/// the claim that survived it. The measurements below are what "the same
/// defect" means numerically.
final class GaitContactClaimTests: XCTestCase {

    private func frames(_ clip: String) throws -> [BodyFrame] {
        try GaitClipFixture.load(clip, bundle: Bundle(for: type(of: self))).frames
    }

    private func report(_ clip: String) throws -> GaitReport {
        try GaitAnalysis.analyse(frames: try frames(clip))
    }

    /// `video_015`'s own measured contact scatter and its own timing floor. The
    /// Monte-Carlo below runs at exactly this configuration so its answer is
    /// about a clip this repo has, not about a chosen one.
    private static let scatterPercent = 11.144
    private static let timingFloorPercent = 8.086
    private static let contactsPerSide = 5
    private static let trials = 20_000

    // MARK: - The false-publication rate, before and after

    /// **THE MEASUREMENT.** A perfectly symmetric runner — both legs' contact
    /// durations drawn from the SAME distribution — filmed so that each side
    /// gives 5 contacts scattering at `video_015`'s measured 11.144 %.
    ///
    /// Nothing about this runner is asymmetric. Under the timing-only floor the
    /// screen prints "Contact time is 9 % longer on the left", unhedged and in
    /// orange, on **one clip in four**.
    ///
    /// Measured, 20 000 seeded trials:
    ///
    /// | gate | publishes on |
    /// |---|---|
    /// | timing floor alone (shipped until 2026-08-08) | **25.3 %** |
    /// | `max(timing floor, sampling half-width)` | **2.4 %** |
    ///
    /// against a 5 % nominal. The new rate is BELOW nominal, not at it, for the
    /// reason pinned in `testTheDegreesOfFreedomChoiceIsConservativeNotNominal`.
    func testASymmetricRunnerPublishedAContactFindingOnAQuarterOfClipsAndNowDoesNot() {
        let result = Self.publicationRate(trueAsymmetryPercent: 0)
        print("GAIT-METRIC contact_claim_false_publication trials=\(Self.trials) "
              + "scatter_percent=\(Self.scatterPercent) contacts_per_side=\(Self.contactsPerSide) "
              + "timing_floor_percent=\(Self.timingFloorPercent) "
              + "old_rate=\(result.timingOnly) new_rate=\(result.withSampling) "
              + "timing_floor_binds=\(result.timingFloorBinds)")

        // The defect, reproduced. The reviewer's figure was 25.3 %; this
        // generator gives the same number to within its own Monte-Carlo error.
        XCTAssertEqual(result.timingOnly, 0.2529, accuracy: 0.005,
                       "a symmetric runner cleared the timing-only floor on a quarter of clips")

        // The fix, measured. It must land AT OR BELOW the nominal level — the
        // point is that the gate now contains the scatter of the thing it gates.
        XCTAssertLessThanOrEqual(result.withSampling, GaitAnalysis.contactClaimErrorRate,
                                 "the corrected gate must not exceed its own nominal level: "
                                 + "\(result.withSampling)")
        XCTAssertEqual(result.withSampling, 0.0243, accuracy: 0.005)

        // And the improvement is an order of magnitude, not a trim.
        XCTAssertLessThan(result.withSampling * 8, result.timingOnly)

        // The term that binds is the sampling one. The timing floor is the
        // larger on well under 5 % of draws, which is the whole point: this
        // claim is limited by the RUNNER, and no frame rate moves that.
        XCTAssertLessThan(result.timingFloorBinds, 0.05,
                          "the contact scatter, not the sampling grid, is what governs: "
                          + "\(result.timingFloorBinds)")
    }

    /// The control. A gate that always refused would pass the test above, so
    /// this one shows the gate still finds a difference that is really there —
    /// and states, in the same numbers, how big it has to be.
    ///
    /// At 5 contacts a side and 11.144 % contact scatter: a true 10 % asymmetry
    /// is published on 14 % of clips, a true 25 % on 76 %, a true 40 % on 99 %.
    /// **So the honest sensitivity of this product's remaining claim is roughly
    /// a 20-25 % left/right contact difference on a 4 s clip.** That is a large
    /// asymmetry. It is also what 5 samples a side buys.
    func testARealAsymmetryStillPublishesAndTheSizeItNeedsIsStated() {
        var rates: [Double] = []
        for planted in [10.0, 25.0, 40.0] {
            let r = Self.publicationRate(trueAsymmetryPercent: planted)
            rates.append(r.withSampling)
            print("GAIT-METRIC contact_claim_power planted_percent=\(planted) "
                  + "old_rate=\(r.timingOnly) new_rate=\(r.withSampling)")
        }
        XCTAssertEqual(rates[0], 0.1449, accuracy: 0.008, "a true 10 % difference: 14 % of clips")
        XCTAssertEqual(rates[1], 0.7554, accuracy: 0.01, "a true 25 % difference: 76 %")
        XCTAssertGreaterThan(rates[2], 0.97, "a true 40 % difference is found essentially always")
        // Monotone in the size of the real effect — a gate that were not would
        // be measuring something other than the difference.
        XCTAssertLessThan(rates[0], rates[1])
        XCTAssertLessThan(rates[1], rates[2])
    }

    /// The degrees of freedom are `min(n_L, n_R) − 1`, matching the muscle path,
    /// and that is a CHOICE with a measured cost: it publishes on 2.4 % where
    /// Welch–Satterthwaite lands at 4.1 % against the same 5 % nominal.
    ///
    /// Pinned so the conservatism is visible rather than assumed. If someone
    /// later wants the nominal level exactly, this is the line to change — and
    /// it moves the false-publication rate UP, which is the direction that
    /// needs an argument.
    func testTheDegreesOfFreedomChoiceIsConservativeNotNominal() {
        let shipped = Self.publicationRate(trueAsymmetryPercent: 0).withSampling
        let welch = Self.publicationRate(trueAsymmetryPercent: 0, useWelchDegreesOfFreedom: true)
            .withSampling
        print("GAIT-METRIC contact_claim_df_choice shipped_min_df=\(shipped) welch_df=\(welch) "
              + "nominal=\(GaitAnalysis.contactClaimErrorRate)")
        XCTAssertLessThan(shipped, welch, "min(n)−1 is the conservative choice")
        XCTAssertEqual(welch, 0.0401, accuracy: 0.005)
        XCTAssertLessThanOrEqual(welch, 0.05, "Welch tracks the nominal level from below")
    }

    // MARK: - The pinned clips

    /// **What the three pinned clips do under the corrected gate, stated
    /// plainly: none of them publishes a contact-time claim, and none of them
    /// did before either.**
    ///
    /// This is the owner's whole remaining deliverable, so the emptiness is
    /// asserted rather than left to be discovered:
    ///
    /// | clip | measured | timing floor | sampling half-width | claim floor | claim |
    /// |---|---|---|---|---|---|
    /// | `video_012` | 2.90 % | 10.145 % | 7.451 % | 10.145 % | none |
    /// | `video_015` | −0.54 % | 8.086 % | **16.464 %** | **16.464 %** | none |
    /// | `video_013` | 5.57 % | 18.909 % | 62.031 % | 62.031 % | refused (unusable) |
    ///
    /// The change does not take a claim away from any of them — their measured
    /// asymmetries are far under even the old floor. What it removes is the
    /// one-in-four false finding on clips this fixture does not contain, and
    /// what it reveals is that `video_015`'s real floor is **double** what the
    /// screen was printing: 16.5 %, not 8.1 %.
    func testNoPinnedClipPublishesAContactTimeClaimAndTheFloorIsNowTheLargerTerm() throws {
        for clip in ["video_012", "video_013", "video_015"] {
            let r = try report(clip)
            print("GAIT-METRIC contact_claim_pinned clip=\(clip) usable=\(r.isUsable) "
                  + "measured_percent=\(r.contactAsymmetryPercent) "
                  + "timing_floor=\(r.resolution.resolvableAsymmetryPercent) "
                  + "sampling_half_width=\(r.contactSamplingUncertaintyPercent) "
                  + "claim_floor=\(r.contactClaimFloorPercent) "
                  + "contact_cv_left=\(r.contactVariationPercent.left) "
                  + "contact_cv_right=\(r.contactVariationPercent.right)")
            XCTAssertNil(r.asymmetryClaim, "\(clip) must publish no contact-time claim")
            // The floor contains BOTH terms and is at least the timing one, so
            // the change can only ever refuse more, never publish more.
            XCTAssertGreaterThanOrEqual(r.contactClaimFloorPercent,
                                        r.resolution.resolvableAsymmetryPercent, clip)
        }
        // On the two usable clips the sampling term is real and finite — a
        // floor that came back infinite everywhere would also pass the
        // assertions above while meaning something quite different.
        for clip in ["video_012", "video_015"] {
            let r = try report(clip)
            XCTAssertTrue(r.contactSamplingUncertaintyPercent.isFinite, clip)
            XCTAssertGreaterThan(r.contactSamplingUncertaintyPercent, 0, clip)
        }
    }

    /// The number the report publishes IS the shared estimator applied to the
    /// clip's own contact durations — not a second implementation that could
    /// drift from the muscle path's.
    func testThePublishedHalfWidthIsTheSharedEstimatorOnTheClipsOwnContacts() throws {
        for clip in ["video_012", "video_015"] {
            let r = try report(clip)
            let expected = MeanDifferenceUncertainty.halfWidthPercent(
                left: r.stance.left.map(\.seconds),
                right: r.stance.right.map(\.seconds),
                alpha: GaitAnalysis.contactClaimErrorRate)
            XCTAssertEqual(r.contactSamplingUncertaintyPercent, expected, accuracy: 1e-12, clip)
            // And it is the same function the muscle path calls, at the same
            // α with one comparison instead of ~175.
            XCTAssertEqual(expected,
                           GaitLoadSummary.samplingUncertaintyPercent(
                               left: r.stance.left.map(\.seconds),
                               right: r.stance.right.map(\.seconds),
                               comparisons: 1),
                           accuracy: 1e-12, clip)
        }
    }

    /// Half-frame edge quantisation does NOT explain `video_015`'s contact
    /// scatter, which is the sentence the old floor rested on.
    ///
    /// Two independent ±½-frame edges give a duration sd of `√(2/12) = 0.408`
    /// frames; over 6.1833 frames per contact that is 6.60 % of CV against
    /// 11.144 % measured. In quadrature, **65 % of the variance is not edge
    /// jitter** — it is the runner varying his own contact times, and no
    /// quantisation floor counts it.
    func testEdgeQuantisationExplainsOnlyAThirdOfTheContactScatter() throws {
        let r = try report("video_015")
        let measured = largerFinite(r.contactVariationPercent.left,
                                    r.contactVariationPercent.right)
        XCTAssertEqual(measured, Self.scatterPercent, accuracy: 0.05)
        let quantisationCV = 100 * (2.0 / 12.0).squareRoot() / r.resolution.framesPerContact
        let unexplainedShareOfVariance = 1 - (quantisationCV * quantisationCV)
            / (measured * measured)
        print("GAIT-METRIC contact_scatter_decomposition measured_cv=\(measured) "
              + "quantisation_cv=\(quantisationCV) "
              + "unexplained_variance_share=\(unexplainedShareOfVariance)")
        XCTAssertEqual(quantisationCV, 6.60, accuracy: 0.05)
        XCTAssertGreaterThan(unexplainedShareOfVariance, 0.6,
                             "most of the contact scatter is the runner, not the grid")
    }

    // MARK: - Gate invariants

    /// A refused claim always leaves a flag, and the flag carries the WHOLE
    /// floor. The two were built from different numbers for one commit's worth
    /// of drafting: the claim from `max(timing, sampling)` and the flag from
    /// `timing`, so a clip could lose its claim and get no explanation.
    func testARefusedClaimAlwaysLeavesAFlagCarryingTheSameFloorItWasRefusedAgainst() throws {
        for clip in ["video_012", "video_015"] {
            let r = try report(clip)
            XCTAssertNil(r.asymmetryClaim, clip)
            let flagged = r.flags.compactMap { flag -> Double? in
                if case .asymmetryBelowResolution(_, let floor) = flag { return floor }
                return nil
            }
            XCTAssertEqual(flagged.count, 1, "\(clip) must explain the refusal exactly once")
            XCTAssertEqual(flagged[0], r.contactClaimFloorPercent, accuracy: 1e-12, clip)
        }
    }

    /// A side with fewer than two contacts has no scatter to estimate, so the
    /// half-width is infinite and NOTHING can be published — the failure mode
    /// being avoided is reading "no scatter measured" as "no scatter".
    func testOneContactOnASideMakesTheHalfWidthInfiniteRatherThanZero() {
        let alpha = GaitAnalysis.contactClaimErrorRate
        XCTAssertFalse(MeanDifferenceUncertainty
            .halfWidthPercent(left: [0.2], right: [0.2, 0.21, 0.19], alpha: alpha).isFinite)
        XCTAssertFalse(MeanDifferenceUncertainty
            .halfWidthPercent(left: [], right: [], alpha: alpha).isFinite)
        // Two identical contacts a side: zero measured scatter is a legitimate
        // zero, not an infinity — the estimator must not refuse everything.
        XCTAssertEqual(MeanDifferenceUncertainty
            .halfWidthPercent(left: [0.2, 0.2], right: [0.2, 0.2], alpha: alpha), 0,
                       accuracy: 1e-12)
    }

    // MARK: - The Monte-Carlo

    private struct Rate {
        var timingOnly: Double
        var withSampling: Double
        var timingFloorBinds: Double
    }

    /// Draws `trials` symmetric-or-planted clips and reports how often each gate
    /// publishes. The arithmetic is the shipping arithmetic:
    /// `contactAsymmetryPercent` is `100·(mL − mR)/mean` and the floor is
    /// `max(resolvableAsymmetryPercent, MeanDifferenceUncertainty.halfWidthPercent)`.
    ///
    /// It runs on DURATIONS rather than on synthetic `BodyFrame`s deliberately:
    /// the question is the gate's error rate at a stated scatter, and putting a
    /// stance detector in front of it would make the answer a statement about
    /// the detector. `testThePublishedHalfWidthIsTheSharedEstimatorOnTheClips…`
    /// is what ties this arithmetic back to what `GaitReport` publishes.
    private static func publicationRate(trueAsymmetryPercent delta: Double,
                                        useWelchDegreesOfFreedom: Bool = false) -> Rate {
        var rng = GaitLoadStatisticTests.Deterministic(seed: 20260808)
        let mean = 0.200
        let sigma = scatterPercent / 100 * mean
        let n = contactsPerSide
        var timingOnly = 0, withSampling = 0, floorBinds = 0

        for _ in 0..<trials {
            let left = (0..<n).map { _ in
                rng.normal(mean: mean * (1 + delta / 200), sigma: sigma)
            }
            let right = (0..<n).map { _ in
                rng.normal(mean: mean * (1 - delta / 200), sigma: sigma)
            }
            let mL = MeanDifferenceUncertainty.mean(left)
            let mR = MeanDifferenceUncertainty.mean(right)
            let m = 0.5 * (mL + mR)
            let asymmetry = 100 * (mL - mR) / m

            if abs(asymmetry) >= timingFloorPercent { timingOnly += 1 }

            let half: Double
            if useWelchDegreesOfFreedom {
                let vL = MeanDifferenceUncertainty.variance(left)
                let vR = MeanDifferenceUncertainty.variance(right)
                let se2 = vL / Double(n) + vR / Double(n)
                let df = Int(se2 * se2
                    / (pow(vL / Double(n), 2) / Double(n - 1)
                        + pow(vR / Double(n), 2) / Double(n - 1)))
                half = MeanDifferenceUncertainty.tMultiplier(
                    degreesOfFreedom: Swift.max(df, 1),
                    alpha: GaitAnalysis.contactClaimErrorRate)
                    * 100 * se2.squareRoot() / m
            } else {
                half = MeanDifferenceUncertainty.halfWidthPercent(
                    left: left, right: right, alpha: GaitAnalysis.contactClaimErrorRate)
            }
            let floor = Swift.max(timingFloorPercent, half)
            if floor == timingFloorPercent { floorBinds += 1 }
            if abs(asymmetry) >= floor { withSampling += 1 }
        }
        return Rate(timingOnly: Double(timingOnly) / Double(trials),
                    withSampling: Double(withSampling) / Double(trials),
                    timingFloorBinds: Double(floorBinds) / Double(trials))
    }
}

extension GaitLoadStatisticTests.Deterministic {
    /// An UNCLAMPED Normal(`mean`, `sigma`). `activation(mean:sigma:floor:)`
    /// clamps to the QP's box, which is right for an activation and wrong for a
    /// contact duration — clamping would truncate the very tail whose
    /// probability this file is measuring.
    mutating func normal(mean: Double, sigma: Double) -> Double {
        let u1 = Swift.max(uniform(), 1e-12), u2 = uniform()
        let z = (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
        return mean + sigma * z
    }
}
