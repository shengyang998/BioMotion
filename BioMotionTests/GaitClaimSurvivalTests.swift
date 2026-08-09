import XCTest
@testable import BioMotion

/// **How many per-muscle left/right claims survive once the interval holds over
/// the whole pool it was selected from** — measured on the three pinned clips,
/// per clip, and reported whatever the answer is.
///
/// # The defect this measures
///
/// `GaitLoadSummary.make` builds one `MuscleLoad` for EVERY bilateral pair the
/// model carries — 175 of `FullBody.osim`'s 520 muscles have both an `_l` and an
/// `_r` — `ordered(_:)` sorts them by `|differencePercent| / claimFloorPercent`,
/// and the panel drew the top eight under the caption "Each comparison is a 95 %
/// one and 8 are shown, so about one in twenty of them can read a difference
/// that is not there". Both halves are wrong in the same direction: the family
/// is 175 and not 8, and the eight shown are the largest ORDER STATISTICS of the
/// very quantity the interval is about, so the false ones sort to the top by
/// construction.
///
/// # What is real here and what is not
///
/// The clips are real: `GaitAnalysis` runs on the pinned fixtures and supplies
/// each clip's own contact counts, timing floor and contact-time term.
///
/// The ACTIVATIONS are synthetic, and they have to be: the fixtures carry five
/// joints (pelvis, ankles, toes), which is what a gait analysis needs and not
/// what an IK solve needs, so no per-muscle activation exists for these clips at
/// all. What is drawn here is contact-to-contact SCATTER — the quantity the
/// sampling interval is built from — at four levels, the largest of which
/// (`σ = 0.12` of activation) is the one `GaitLoadStatisticTests` uses for every
/// other measurement in this repo. Both legs are drawn from the SAME
/// distribution, so every survivor counted below is a FALSE finding.
///
/// The headline is not sensitive to that choice, which is the point: above the
/// scatter at which the sampling term starts to bind, the per-comparison rule
/// admits α of the pool BY CONSTRUCTION — a t-interval is calibrated, so ~5 % of
/// 175 comparisons clear their own floor however noisy the muscles are.
final class GaitClaimSurvivalTests: XCTestCase {

    private var bundle: Bundle { Bundle(for: type(of: self)) }

    /// Bilateral pairs in the shipping model. Not a guess: `MomentArmTests`
    /// parses `FullBody.osim` for the wrap table, and this is the same file's
    /// count of muscles carrying both an `_l` and an `_r`.
    private static let bilateralPairsInFullBody = 175

    /// Contact-to-contact scatter of one muscle's activation, as a standard
    /// deviation on a mean of 0.50. The last is the repo's own harness value.
    private static let scatterLevels = [0.02, 0.04, 0.08, 0.12]

    // MARK: - The measurement

    /// **The count, per clip, before and after the correction.**
    func testHowManyClaimsSurviveTheFamilyWiseCorrectionOnEachPinnedClip() throws {
        var anyClipAdmittedAFalseFindingUncorrected = false

        for id in GaitClipFixture.allIds {
            let frames = try GaitClipFixture.load(id, bundle: bundle).frames
            let report = try GaitAnalysis.analyse(frames: frames)
            guard report.isUsable else {
                // `video_013` is the refused clip and stays refused: it never
                // reaches a muscle comparison, so its answer is zero for a
                // reason that has nothing to do with multiplicity.
                print("CLAIM-SURVIVAL clip=\(id) usable=false refusals="
                      + "\(report.refusals.map(\.description)) claims_uncorrected=0 "
                      + "claims_corrected=0")
                continue
            }

            let contacts = Swift.min(report.stance.left.count, report.stance.right.count)
            for sigma in Self.scatterLevels {
                let outcome = try measure(report: report, contactsPerSide: contacts,
                                          scatter: sigma, seed: 20260808)
                print(String(format: "CLAIM-SURVIVAL clip=%@ usable=true contacts_per_side=%d "
                             + "sigma=%.2f screened=%d timing_floor=%.3f contact_time_term=%.3f "
                             + "uncorrected_floor_median=%.1f corrected_floor_median=%.1f "
                             + "claims_uncorrected=%d claims_corrected=%d "
                             + "largest_false_difference=%.1f",
                             id, contacts, sigma, outcome.screened,
                             report.resolution.resolvableAsymmetryPercent,
                             outcome.contactTimeTerm,
                             outcome.medianUncorrectedFloor, outcome.medianCorrectedFloor,
                             outcome.survivorsUncorrected, outcome.survivorsCorrected,
                             outcome.largestDifference))

                // The pool is the model's bilateral muscles, not the eight the
                // panel used to draw.
                XCTAssertEqual(outcome.screened, Self.bilateralPairsInFullBody,
                               "every bilateral pair is screened, so every one is in the family")
                // The correction can only ever remove claims. If it ever added
                // one, the multiplier would be going the wrong way.
                XCTAssertLessThanOrEqual(outcome.survivorsCorrected, outcome.survivorsUncorrected,
                                         "a wider interval cannot admit more claims")
                if outcome.survivorsUncorrected > 0 {
                    anyClipAdmittedAFalseFindingUncorrected = true
                }
                // Nothing false may survive the corrected rule — on a runner
                // whose two legs were drawn from one distribution, every
                // survivor here is a training change made on noise.
                XCTAssertEqual(outcome.survivorsCorrected, 0,
                               "\(id) at σ=\(sigma) published \(outcome.survivorsCorrected) "
                               + "false finding(s) with the family-wise interval")
            }
        }

        // And the control: the old rule DID admit them, or this test would be
        // measuring nothing. This is the "~9 of 175 by chance" the review
        // predicted, on the shipping code path.
        XCTAssertTrue(anyClipAdmittedAFalseFindingUncorrected,
                      "the per-comparison rule must be shown to publish false findings on a "
                      + "symmetric runner, or the correction is fixing nothing")
    }

    /// **What effect size the corrected rule can still see.** Swept rather than
    /// asserted at one value, because the useful output is the threshold: if the
    /// smallest detectable asymmetry is above what a body can produce, the
    /// feature has no operating range at all.
    func testTheSmallestAsymmetryTheCorrectedRuleCanStillDetect() throws {
        for id in GaitClipFixture.allIds {
            let frames = try GaitClipFixture.load(id, bundle: bundle).frames
            let report = try GaitAnalysis.analyse(frames: frames)
            guard report.isUsable else { continue }
            let contacts = Swift.min(report.stance.left.count, report.stance.right.count)

            for sigma in [0.04, 0.12] {
                var detected: Double = .infinity
                // A planted difference, as a percentage of the mean.
                // `differencePercent` is bounded by ±200 % because it divides by
                // the two sides' mean, and the sweep stops at 140 % because the
                // weaker side would otherwise be pushed onto the QP's own
                // resting-tone floor, where the comparison is withheld for a
                // different reason.
                for planted in [10.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0] {
                    let outcome = try measure(report: report, contactsPerSide: contacts,
                                              scatter: sigma, seed: 77, plantedDifference: planted)
                    if outcome.plantedSurvivedCorrected {
                        detected = planted
                        break
                    }
                }
                print(String(format: "CLAIM-SURVIVAL-THRESHOLD clip=%@ sigma=%.2f "
                             + "contacts_per_side=%d smallest_detectable_percent=%@",
                             id, sigma, contacts,
                             detected.isFinite ? String(format: "%.0f", detected)
                                               : "none_at_or_below_140"))
            }
        }
    }

    /// The Bonferroni multiplier is not free, and the size of what it costs is
    /// worth having in one line: at the contact counts a 4 s clip gives, the
    /// interval roughly quadruples.
    func testTheMultiplierAtTheContactCountsARealClipCarries() {
        for df in [3, 4, 5] {
            let one = GaitLoadSummary.tMultiplier(degreesOfFreedom: df, comparisons: 1)
            let family = GaitLoadSummary.tMultiplier(degreesOfFreedom: df,
                                                     comparisons: Self.bilateralPairsInFullBody)
            print(String(format: "CLAIM-SURVIVAL-MULTIPLIER df=%d per_comparison=%.3f "
                         + "family_wise_175=%.3f ratio=%.2f", df, one, family, family / one))
            XCTAssertGreaterThan(family, one)
        }
    }

    // MARK: - Harness

    private struct Outcome {
        var screened = 0
        var survivorsUncorrected = 0
        var survivorsCorrected = 0
        var medianUncorrectedFloor = 0.0
        var medianCorrectedFloor = 0.0
        var largestDifference = 0.0
        var contactTimeTerm = 0.0
        var plantedSurvivedCorrected = false
    }

    /// Runs the SHIPPING summary path on a synthetic muscle population and
    /// counts survivors under both rules.
    ///
    /// The corrected count comes from `GaitLoadSummary` itself
    /// (`clearsStatisticalFloor`), so what is measured is the code that ships.
    /// The uncorrected count is recomputed here from the same per-contact
    /// samples through the same public function with `comparisons: 1` — the
    /// rule that shipped before — so the two differ in exactly one input.
    private func measure(report: GaitReport,
                         contactsPerSide: Int,
                         scatter: Double,
                         seed: UInt64,
                         plantedDifference: Double = 0) throws -> Outcome {
        var rng = GaitLoadStatisticTests.Deterministic(seed: seed)
        let bases = (0..<Self.bilateralPairsInFullBody).map { "m\($0)" }
        let plantedBase = bases[0]
        // A planted difference is applied as a ratio around a mean of 0.5, so
        // `differencePercent` lands on `plantedDifference` in expectation.
        let leftMean = 0.5 * (1 + plantedDifference / 200)
        let rightMean = 0.5 * (1 - plantedDifference / 200)

        var frames: [OfflineResultStore.FrameResult] = []
        var leftSamples: [String: [Double]] = [:]
        var rightSamples: [String: [Double]] = [:]
        var id = 0
        var contactIndex = 0
        for _ in 0..<contactsPerSide {
            for side in [-1, 1] {
                var activations: [String: Double] = [:]
                for base in bases {
                    let mean = base == plantedBase ? (side < 0 ? leftMean : rightMean) : 0.5
                    let value = rng.activation(mean: mean, sigma: scatter)
                    activations["\(base)_l"] = value
                    activations["\(base)_r"] = value
                    if side < 0 {
                        leftSamples[base, default: []].append(value)
                    } else {
                        rightSamples[base, default: []].append(value)
                    }
                }
                frames.append(Self.stanceFrame(id: id, side: side, contactIndex: contactIndex,
                                               activations: activations))
                id += 1
                contactIndex += 1
            }
        }

        let summary = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                         filterTaps: 5))
        var outcome = Outcome()
        outcome.screened = summary.screenedComparisonCount
        outcome.contactTimeTerm = summary.contactTimeContributionPercent
        var uncorrectedFloors: [Double] = []
        var correctedFloors: [Double] = []

        for load in summary.muscles {
            guard let l = leftSamples[load.id], let r = rightSamples[load.id] else { continue }
            let d = abs(load.differencePercent)
            if load.id != plantedBase { outcome.largestDifference = max(outcome.largestDifference, d) }

            let uncorrected = GaitLoadSummary.samplingUncertaintyPercent(left: l, right: r,
                                                                        comparisons: 1)
            let uncorrectedFloor = Swift.max(summary.resolvableAsymmetryPercent, uncorrected)
                + abs(summary.contactTimeContributionPercent)
            let correctedFloor = summary.claimFloorPercent(for: load)
            uncorrectedFloors.append(uncorrectedFloor)
            correctedFloors.append(correctedFloor)

            let survivesUncorrected = d >= uncorrectedFloor
                && !load.isSaturated && !load.isAtActivationFloor && summary.arePublishable
            let survivesCorrected = summary.clearsStatisticalFloor(load)
            if load.id == plantedBase {
                outcome.plantedSurvivedCorrected = survivesCorrected
                continue                       // the planted one is not a false finding
            }
            if survivesUncorrected { outcome.survivorsUncorrected += 1 }
            if survivesCorrected { outcome.survivorsCorrected += 1 }
        }
        outcome.medianUncorrectedFloor = Self.median(uncorrectedFloors)
        outcome.medianCorrectedFloor = Self.median(correctedFloors)
        return outcome
    }

    private static func median(_ values: [Double]) -> Double {
        guard !values.isEmpty else { return .nan }
        let sorted = values.sorted()
        return sorted[sorted.count / 2]
    }

    private static func stanceFrame(id: Int, side: Int, contactIndex: Int,
                                    activations: [String: Double])
        -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.1,
            residualInBodyWeights: 0.1,
            contactSide: side,
            contactIndex: contactIndex,
            solverSawLeftContact: side < 0,
            solverSawRightContact: side > 0,
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: true)
        let muscle = NimbleEngine.MuscleOutput(activations: activations,
                                               forces: activations.mapValues { $0 * 1000 },
                                               converged: true,
                                               timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitStance, outcome: outcome))
    }
}
