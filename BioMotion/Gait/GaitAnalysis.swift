import Foundation

/// What this clip can and cannot resolve, computed from the clip itself.
///
/// The binding limit on every left/right claim this product makes is FRAMES PER
/// CONTACT, not the detector and not the pose model. A contact sampled `N` times
/// has its two edges located to ±½ frame each, so its duration carries a
/// quantisation error of about `0.5/N` in relative terms. Measured against
/// stride-to-stride scatter across the three pinned clips the two track each
/// other (`corr 0.73` over the earlier criterion), and `video_015` — the clip
/// with the longest contacts and the slowest cadence — sits ON the floor.
///
/// So the number this type publishes is the LARGER of two things that both bound
/// a claim from below:
///
/// * `quantisationFloorPercent` — what the sampling grid allows in principle.
/// * `strideRepeatabilityPercent` — what this runner's own strides actually did.
///   A left/right difference smaller than the difference between one stride and
///   the next is not a finding about sides.
///
/// This is a per-clip number the user can act on ("this clip resolves left/right
/// to about ±10 %"), which was chosen over a global disclaimer precisely because
/// it tells them WHEN a faster capture would help. For a 200 ms contact the
/// floor is 8.3 % at 30 fps, 4.2 % at 60, 2.1 % at 120 and 1.0 % at 240.
///
/// # `strideRepeatabilityPercent` is the STRIDE PERIOD's scatter, and only that
///
/// It used to be fed the coefficient of variation of CONTACT DURATIONS, which
/// is a different quantity and is mostly the detector's own edge jitter — the
/// thing a faster camera fixes. Measured on the fixtures: `video_015` reads
/// 11.14 % of contact-duration scatter (contacts of 5,6,6,6,7,7 samples) while
/// the SAME detector on the SAME clip measures its stride period at 2.08 % /
/// 2.56 % (touchdown gaps 19,19,20,19,19). Feeding the first number in made the
/// app tell a `video_015` user that the runner was the limit and a faster
/// camera could not help — on the best of the three clips, and wrongly, because
/// at 6.18 samples per contact one frame IS 16 % and most of that 11 % was the
/// quantisation being counted a second time.
///
/// The published resolution changes accordingly: `video_015` 11.14 % → 8.09 %
/// (its floor), `video_013` 43.28 % → 18.91 %, `video_012` unchanged at
/// 10.15 % because its floor already dominated.
///
/// # `strideRepeatabilityPercent` is FLOORED at what the clip could have seen
///
/// The scatter is a coefficient of variation over touchdown GAPS, and those gaps
/// are whole numbers of frames. On `video_012` every gap is exactly 18 samples,
/// so the CV is exactly 0 — and 0.000 was published to the user as "this runner's
/// own stride-to-stride variation ±0%", a statement about the runner that the
/// clip cannot support. A 30 fps clip with an 18-frame stride cannot distinguish
/// a perfectly steady runner from one varying by up to one sampling interval,
/// i.e. `100/18 = 5.56%` — the same number `GaitSteadiness.boundPercent` already
/// publishes as "the coarsest variation this clip could not have distinguished
/// from a steady runner".
///
/// So the published repeatability is `max(measured, bound)` and the raw
/// measurement is kept beside it as `measuredStrideRepeatabilityPercent`. The
/// consequence that matters is the PROMISE: `bestAchievablePercentAtAnyFrameRate`
/// was returning 0.000 on `video_012`, which let `resolutionSentence` promise
/// "filming at 61 fps would resolve ±5%" on a clip whose own stride scatter could
/// be 5.6% — a promise the faster camera could not keep.
///
/// Measured blast radius, all three pinned clips: the published resolution
/// `max(floor, repeatability)` does not move on any of them (10.145 / 18.909 /
/// 8.086), because the quantisation floor `50/framesPerContact` sits at
/// `stridePeriodFrames / (2·framesPerContact)` times the stride bound
/// `100/stridePeriodFrames` — 1.83, 1.91 and 1.54 on the three clips — and that
/// ratio does not move with capture rate, since both terms scale together. Only
/// the displayed breakdown and the frame-rate promise change.
struct GaitResolution: Equatable {
    let framesPerContact: Double
    let quantisationFloorPercent: Double
    /// What the touchdown gaps actually scattered by — quantised to whole
    /// frames, so it can read exactly 0.
    let measuredStrideRepeatabilityPercent: Double
    /// One sampling interval as a fraction of the stride period: the finest
    /// stride-to-stride variation this clip could have distinguished from a
    /// perfectly steady runner. 0 when the stride period is unknown.
    let strideRepeatabilityBoundPercent: Double
    /// `max(measured, bound)` — what the runner's own strides allow, never finer
    /// than the clip could have seen.
    let strideRepeatabilityPercent: Double
    /// `max` of the grid floor and the repeatability. The finest left/right
    /// difference this clip may assert.
    let resolvableAsymmetryPercent: Double

    /// - Parameter stridePeriodFrames: the detector's stride period in samples.
    ///   Pass 0 only where there is no stride period to speak of — the floor is
    ///   then disabled and the measured scatter is published raw.
    init(framesPerContact: Double,
         strideRepeatabilityPercent measured: Double,
         stridePeriodFrames: Int) {
        self.framesPerContact = framesPerContact
        let floor = framesPerContact > 0 ? 100 * 0.5 / framesPerContact : .infinity
        quantisationFloorPercent = floor
        measuredStrideRepeatabilityPercent = measured
        let bound = stridePeriodFrames > 0 ? 100.0 / Double(stridePeriodFrames) : 0
        strideRepeatabilityBoundPercent = bound
        // A NON-finite measurement means no leg had a stride period at all, and
        // that has to keep poisoning the resolution — the bound must not rescue
        // it into a claimable number. `largerFinite` would have done exactly
        // that.
        let repeatability = measured.isFinite ? Swift.max(measured, bound) : measured
        self.strideRepeatabilityPercent = repeatability
        resolvableAsymmetryPercent = Swift.max(floor, repeatability)
    }

    /// The gate every asymmetry claim has to pass. A difference finer than the
    /// clip's own resolution is refused rather than shown with a caveat.
    func permitsAsymmetryClaim(ofPercent percent: Double) -> Bool {
        percent.isFinite && abs(percent) >= resolvableAsymmetryPercent
    }

    /// The lever, when a claim is refused. Contact duration is a property of the
    /// runner, so the only thing the user can change is the sampling rate.
    ///
    /// ⚠️ This answers "what rate puts the QUANTISATION FLOOR at `percent`", which
    /// is not the same as "what rate lets me claim `percent`" — the published
    /// resolution is `max(floor, strideRepeatability)` and no camera moves the
    /// second term. Ask `bestAchievablePercentAtAnyFrameRate` first; the UI does.
    func framesPerSecondNeeded(for percent: Double, currentFPS: Double) -> Double {
        guard percent > 0, framesPerContact > 0 else { return .infinity }
        let neededFrames = 100 * 0.5 / percent
        return currentFPS * neededFrames / framesPerContact
    }

    /// The finest left/right difference ANY capture rate could support on this
    /// runner. As the rate rises the quantisation floor goes to zero and the
    /// runner's own stride-to-stride variation is what is left, so promising
    /// anything finer than this is promising something the camera cannot buy.
    ///
    /// Reads the FLOORED repeatability, not the raw measurement — a CV over
    /// frame-quantised gaps that happens to land on 0.000 is not evidence that a
    /// faster camera would find nothing there.
    var bestAchievablePercentAtAnyFrameRate: Double {
        strideRepeatabilityPercent.isFinite ? strideRepeatabilityPercent : .infinity
    }
}

/// Whether the periodicity the whole route rests on actually holds on this clip.
///
/// `Fmax = m·g·(π/2)(1 + tf/tc)` is a statement about a REPEATING stride: it
/// closes the vertical impulse over one cycle by assuming the next cycle is like
/// this one. If the strides are not alike, `tf/tc` is an average over unlike
/// things and the peak force it implies belongs to no actual step.
///
/// The bound is the frame rate, not a taste: one sampling interval of stride
/// period is the coarsest variation this clip could not have distinguished from
/// a perfectly steady runner, so `bound = 1 / stridePeriodFrames`. At the pinned
/// clips' 18-19 frame strides that is 5.3-5.6 %.
///
/// **The failing value, stated:** `video_013`'s RIGHT leg varies its stride
/// period by 18.91 % — 3.4 frames — against a bound of 5.56 %. `video_012`
/// measures 0.00 % on both legs and `video_015` 2.08 % / 2.56 %.
struct GaitSteadiness: Equatable {
    let strideVariationPercent: Bilateral<Double>
    let boundPercent: Double
    let isSteady: Bool

    init(strideVariationPercent: Bilateral<Double>, stridePeriodFrames: Int) {
        self.strideVariationPercent = strideVariationPercent
        let bound = stridePeriodFrames > 0 ? 100.0 / Double(stridePeriodFrames) : .infinity
        boundPercent = bound
        isSteady = strideVariationPercent.left.isFinite
            && strideVariationPercent.right.isFinite
            && strideVariationPercent.left <= bound
            && strideVariationPercent.right <= bound
    }
}

/// The whole result of a gait analysis: what was measured, what may be claimed,
/// and what — if anything — contradicted the model.
struct GaitReport {

    /// A reason to withhold the clip's force and asymmetry output entirely.
    /// Every case carries the number that produced it.
    enum Refusal: Equatable, CustomStringConvertible {
        case tooFewContacts(side: GaitSide, count: Int)
        case stridePeriodDisagreesBetweenLegs(frames: Double)
        /// ⚠️ **Not a force falsifier, and it must never be described as one.**
        /// See `GaitReport.contactSequencePeriodicityErrorFrames`: for any
        /// perfectly periodic alternating schedule the two flight estimates are
        /// algebraically the SAME number, whatever the contact durations and
        /// therefore whatever `Fmax` is. What this refusal catches is a contact
        /// sequence that is not periodic and alternating.
        case contactSequenceNotPeriodic(frames: Double)
        case strideNotSteady(side: GaitSide, percent: Double, boundPercent: Double)
        case notRunning(dutyFactor: Double, flightToContactRatio: Double)
        case stanceBudgetInconsistent(assumed: Int, measured: Int)
        case contactTooShortToResolve(framesPerContact: Double)
        /// The decoder lost slots inside or against the edge of a contact, so
        /// that contact's duration is not resolved to what this clip publishes.
        case droppedSamplesInContact(side: GaitSide, inside: Int, atEdges: Int)
        /// The median contact holds fewer samples than the shortest usable
        /// centred derivative window, so no stance frame can carry an
        /// acceleration that was not fitted across a touchdown or a toe-off.
        case contactTooShortForACleanDerivative(medianSamples: Int, neededSamples: Int)

        var description: String {
            switch self {
            case .tooFewContacts(let side, let count):
                return "\(side.rawValue): only \(count) complete contacts in this window"
            case .stridePeriodDisagreesBetweenLegs(let frames):
                return String(format: "the two legs report stride periods %.2f frames apart; "
                              + "in steady gait they alternate within one cycle and must agree", frames)
            case .contactSequenceNotPeriodic(let frames):
                return String(format: "the contact sequence is not periodic and alternating: flight "
                              + "measured between contacts and flight implied by closing the stride "
                              + "differ by %.2f frames", frames)
            case .strideNotSteady(let side, let percent, let bound):
                return String(format: "%@ stride period varies %.2f%% (bound %.2f%%); the periodic "
                              + "force model assumes one stride is like the next",
                              side.rawValue, percent, bound)
            case .notRunning(let duty, let ratio):
                return String(format: "duty factor %.3f, flight/contact %.3f — no flight phase, so the "
                              + "half-sine impulse model does not apply", duty, ratio)
            case .stanceBudgetInconsistent(let assumed, let measured):
                return "the plateau level was estimated over \(assumed) frames per cycle but the "
                     + "contacts measure \(measured); the level's support is the wrong length"
            case .contactTooShortToResolve(let n):
                return String(format: "%.2f frames per contact — too few to time an edge", n)
            case .droppedSamplesInContact(let side, let inside, let atEdges):
                return "the video lost \(inside) frame(s) inside and \(atEdges) against the edge of "
                     + "a \(side.rawValue) contact; a contact with a hole is not timed to what this "
                     + "clip resolves. Re-film in better light, or hold the subject in frame"
            case .contactTooShortForACleanDerivative(let median, let needed):
                return "the median contact holds \(median) frames and a derivative window needs "
                     + "\(needed); every stance acceleration would be fitted across a touchdown. "
                     + "Film at a higher frame rate"
            }
        }
    }

    /// Something the user should be told that does not withhold the result.
    enum Flag: Equatable, CustomStringConvertible {
        case droppedFrames(count: Int, largestGapInFrames: Int)
        case edgeClippedRunsExcluded(count: Int)
        case asymmetryBelowResolution(measuredPercent: Double, resolvablePercent: Double)

        var description: String {
            switch self {
            case .droppedFrames(let count, let gap):
                return "\(count) frames carry no pose (largest gap \(gap) frames)"
            case .edgeClippedRunsExcluded(let count):
                return "\(count) contact(s) ran past the end of the analysis window and were dropped"
            case .asymmetryBelowResolution(let measured, let resolvable):
                return String(format: "left/right contact differs by %.2f%%, below what this clip "
                              + "resolves (%.2f%%) — not a finding", measured, resolvable)
            }
        }
    }

    // Sampling
    let frameCount: Int
    let sampleInterval: Double
    var framesPerSecond: Double { 1 / sampleInterval }
    let droppedFrameCount: Int
    let largestGapInFrames: Int

    // Detector state, published so a reader can check the level it used
    let stridePeriodFrames: Int
    let stridePeriodCorrelation: Double
    let runningSpeedMetersPerSecond: Double
    let plateauScatterMetersPerSecond: Double
    let detectionLevelMetersPerSecond: Double
    let stanceFrameBudget: Int
    let measuredStanceFrames: Int

    // Events
    let stance: Bilateral<[StanceInterval]>
    let edgeClipped: Bilateral<[StanceInterval]>
    let contactSeconds: Bilateral<Double>
    let contactFrames: Bilateral<Double>
    let contactVariationPercent: Bilateral<Double>
    let strideSeconds: Bilateral<Double>
    let strideVariationPercent: Bilateral<Double>

    /// Flight time as OBSERVED: the gap between one foot leaving the ground and
    /// the other landing.
    let measuredFlightSeconds: Double
    let measuredFlightScatterSeconds: Double
    /// Flight time as IMPLIED by closing the stride: `(T − tcL − tcR)/2`.
    let modelledFlightSeconds: Double

    /// How far the contact SEQUENCE is from being periodic and alternating, in
    /// sampling intervals. Measured 0.0119 on `video_012`, 0.0083 on
    /// `video_015`, 0.8994 on `video_013`.
    ///
    /// # ⚠️ This is NOT a falsifier of the force model, and here is the proof
    ///
    /// For touchdowns at `L: nT` and `R: nT + s` with contacts `cL`, `cR`, the
    /// observed gaps are `s − cL` and `T − s − cR`, whose mean is exactly
    /// `(T − cL − cR)/2` — which is the modelled flight, identically, for EVERY
    /// `s`, `cL` and `cR`. So on a periodic alternating schedule the two
    /// estimates are the same number and this quantity is 0 by algebra, while
    /// `Fmax = (π/2)(1 + tf/tc)` sweeps 7.07 → 1.18 BW as the contacts vary.
    /// `GaitReportTests.testThePeriodicityCheckIsAnIdentityAndCannotSeeTheForce`
    /// asserts both halves of that.
    ///
    /// What it DOES test is that the detected events form a periodic,
    /// alternating sequence — a real property that `video_013` fails. It is
    /// kept, and named for what it measures.
    var contactSequencePeriodicityErrorFrames: Double {
        abs(modelledFlightSeconds - measuredFlightSeconds) / sampleInterval
    }

    /// The whole-clip force model. `dutyFactor` and `describesRunning` are
    /// clip-level questions and read from here.
    let force: GaitForceModel
    /// Peak vertical GRF per leg, body weights — each closed on its own contact
    /// time `Fmax_side = (π/2)(1 + tf/tc_side)` **where this clip can resolve the
    /// difference between the two contact times**, and on the MEAN contact time
    /// where it cannot. `peakVerticalForceIsSharedBetweenLegs` says which
    /// happened. See `GaitForceModel.perLegPeaksInBodyWeights` for why one gate
    /// has to govern both the displayed timing claim and this force scale.
    ///
    /// One clip-wide peak applied to both feet unconditionally would set the
    /// timing model's own left/right peak-force asymmetry to zero by
    /// construction, and the product exists to find that asymmetry. It also runs
    /// the wrong way round: at a fixed step frequency a SHORTER contact must
    /// carry a HIGHER peak, because it has less time to deliver the same
    /// impulse.
    ///
    /// Cost of getting it wrong in that direction, measured: `tcL = 200 ms`,
    /// `tcR = 160 ms`, `tf = 130 ms` gives 2.7053 BW on both feet under the
    /// shared model against 2.5918 / 2.8471 per leg — a 9.4 % peak asymmetry
    /// discarded, above the 8.3 % floor a 6-frame contact allows. That schedule
    /// is resolvable, so it still gets per-leg peaks.
    ///
    /// Cost of getting it wrong in the OTHER direction, also measured: on
    /// `video_012` the contact difference is 2.899 % against a 10.145 % floor,
    /// and per-leg peaks were injecting −1.31 % of force scale into all 520
    /// muscles' left/right comparisons on a screen that had just refused the
    /// 2.899 % as unresolvable.
    ///
    /// The stride impulse closes exactly either way: `Σ F_i·2tc_i/π = T`,
    /// verified numerically in `GaitReportTests` for both regimes.
    let peakVerticalForceInBodyWeights: Bilateral<Double>
    /// True when the two legs' peaks were closed on the MEAN contact time
    /// because the clip could not resolve the difference between them. Published
    /// rather than inferred from equality: "the legs measured the same" and
    /// "this clip refused to distinguish them" are different statements.
    let peakVerticalForceIsSharedBetweenLegs: Bool
    let resolution: GaitResolution
    let steadiness: GaitSteadiness

    let refusals: [Refusal]
    let flags: [Flag]

    var isUsable: Bool { refusals.isEmpty }

    /// The measured left/right difference in contact time, as a percentage of
    /// their mean. **Only meaningful when `resolution.permitsAsymmetryClaim`
    /// says so** — read `asymmetryClaim` instead of this.
    var contactAsymmetryPercent: Double {
        let m = 0.5 * (contactSeconds.left + contactSeconds.right)
        guard m > 0 else { return .nan }
        return 100 * (contactSeconds.left - contactSeconds.right) / m
    }

    /// The user-facing answer to "am I even?" — `nil` when the clip cannot
    /// resolve the difference it measured, which is the honest answer far more
    /// often than not at 30 fps.
    var asymmetryClaim: Double? {
        guard isUsable,
              resolution.permitsAsymmetryClaim(ofPercent: contactAsymmetryPercent) else { return nil }
        return contactAsymmetryPercent
    }

    /// Samples in the shortest and the median contact of this clip. Measured:
    /// `video_012` 4 / 5, `video_015` 5 / 6, `video_013` 1 / 5.
    let shortestContactSamples: Int
    let medianContactSamples: Int

    /// The centred, odd-tap derivative window this clip gets, sized from the
    /// MEDIAN contact rather than the shortest one.
    ///
    /// The engine's shipped Savitzky-Golay filter is 9 taps, spanning
    /// `8·dt = 267 ms` at 30 fps — LONGER than every contact measured here
    /// (133-233 ms), so no stance frame has a neighbourhood free of a
    /// touchdown/toe-off discontinuity (0 of 114 interior frames on
    /// `video_012`) and its second-derivative gain is 0.49 at the step
    /// fundamental, inverting sign above 7 Hz. A shorter window fixes the bias
    /// and pays in noise, and the exact price is arithmetic — the white-noise
    /// gain of the second-derivative coefficients:
    ///
    ///     taps  order  ‖c_acc‖    vs 9-tap   position coefficients
    ///       9     3    0.113961     1.00×    smoothing
    ///       7     2    0.218218     1.91×    smoothing
    ///       5     2    0.534522     4.69×    smoothing
    ///       3     2    2.449490    21.49×    [0, 1, 0] — NO smoothing at all
    ///
    /// Sizing from the SHORTEST contact picks 3 taps on `video_012` — a 21.5×
    /// amplification of acceleration noise that is INDEPENDENT PER DOF, so
    /// unlike a peak-force error it does not cancel out of a muscle-to-muscle
    /// ratio, which is the one error the product's whole ratio argument is
    /// allowed to tolerate. And at 3 taps the "smoothed" pose the moment-arm
    /// and muscle-length stage consumes is the raw IK output, unsmoothed.
    ///
    /// Sizing from the MEDIAN gives 5 taps on both usable clips (4.69× noise,
    /// genuine smoothing) and leaves the contacts shorter than the window to be
    /// handled where they belong — per frame, by refusing the stance frames
    /// whose own window crosses an edge. See
    /// `NimbleEngine.GaitPlan.Frame.derivativeWindowInsideContact`. Measured
    /// stance frames that keep a clean window: `video_012` 12 of 64,
    /// `video_015` 24 of 68.
    let derivativeFilterTaps: Int
    /// `8·dt` — what the engine's 9-tap window actually spans on this clip.
    var nineTapFilterSpanSeconds: Double { 8 * sampleInterval }
    var nineTapFilterFitsOneContact: Bool { medianContactSamples >= 9 }
}

/// The pure entry point. No solver, no Core ML, no UI, no I/O: an array of
/// frames in, a report out.
enum GaitAnalysis {

    /// A leg needs this many complete contacts before its scatter means
    /// anything. Below 3 the standard deviation of the contact durations is
    /// dominated by which two strides happened to land inside the window.
    static let minimumContactsPerSide = 3
    /// Both falsifiers are compared against one sampling interval: it is the
    /// finest disagreement this clip could have detected at all.
    static let maximumDisagreementFrames = 1.0

    static func analyse(frames: [BodyFrame]) throws -> GaitReport {
        let signal = try GaitSignal.build(frames: frames)
        let detected = StanceDetector.detect(signal)
        let dt = signal.sampleInterval

        var contactSeconds = Bilateral<Double>(left: .nan, right: .nan)
        var contactFrames = Bilateral<Double>(left: .nan, right: .nan)
        var contactVariation = Bilateral<Double>(left: .nan, right: .nan)
        var strideSeconds = Bilateral<Double>(left: .nan, right: .nan)
        var strideVariation = Bilateral<Double>(left: .nan, right: .nan)
        var allStanceFrames: [Double] = []

        for side in GaitSide.allCases {
            let intervals = detected.stance[side]
            // Durations off the CLOCK. Counting surviving samples subtracts one
            // sampling interval for every frame the decoder lost inside a
            // contact and turns that into a left/right finding — see
            // `StanceInterval`.
            let seconds = intervals.map(\.seconds)
            allStanceFrames.append(contentsOf: intervals.map { Double($0.samples) })
            contactSeconds[side] = mean(seconds)
            contactFrames[side] = mean(seconds) / dt
            contactVariation[side] = 100 * coefficientOfVariation(seconds)
            let touchdowns = intervals.map(\.touchdown)
            if touchdowns.count > 1 {
                let strides = (1..<touchdowns.count).map { touchdowns[$0] - touchdowns[$0 - 1] }
                strideSeconds[side] = mean(strides)
                strideVariation[side] = 100 * coefficientOfVariation(strides)
            }
        }

        // Flight, measured: order every contact by touchdown and take the gap
        // between an opposite-footed neighbour pair. `+ dt` because the stored
        // toe-off is the LAST stance SAMPLE — the foot is still down at that
        // instant and leaves during the following interval.
        var ordered = (detected.stance.left + detected.stance.right)
        ordered.sort { $0.touchdown < $1.touchdown }
        var flights: [Double] = []
        for (a, b) in zip(ordered, ordered.dropFirst()) where a.side != b.side {
            flights.append(b.touchdown - a.lastStanceSample - dt)
        }
        let measuredFlight = mean(flights)
        let flightScatter = standardDeviation(flights)

        let meanStride = 0.5 * (strideSeconds.left + strideSeconds.right)
        let modelledFlight = 0.5 * (meanStride - contactSeconds.left - contactSeconds.right)
        let meanContact = 0.5 * (contactSeconds.left + contactSeconds.right)
        let force = GaitForceModel(contactSeconds: meanContact, flightSeconds: modelledFlight)

        let framesPerContact = 0.5 * (contactFrames.left + contactFrames.right)
        // The RUNNER's own repeatability is the scatter of the STRIDE PERIOD.
        // Contact-duration scatter is mostly detector edge jitter, which the
        // quantisation floor already counts — see `GaitResolution`.
        let repeatability = largerFinite(strideVariation.left, strideVariation.right)
        let resolution = GaitResolution(framesPerContact: framesPerContact,
                                        strideRepeatabilityPercent: repeatability,
                                        stridePeriodFrames: detected.stridePeriodFrames)
        let steadiness = GaitSteadiness(strideVariationPercent: strideVariation,
                                        stridePeriodFrames: detected.stridePeriodFrames)
        // Each leg's peak is closed on ITS OWN contact time — but only where
        // this clip can resolve the contact difference that separates them. See
        // `GaitForceModel.perLegPeaksInBodyWeights`.
        let peaks = GaitForceModel.perLegPeaksInBodyWeights(
            contactSeconds: contactSeconds,
            flightSeconds: modelledFlight,
            resolvableAsymmetryPercent: resolution.resolvableAsymmetryPercent)

        let measuredStanceFrames = allStanceFrames.isEmpty ? 0 : Int(median(allStanceFrames).rounded())
        let shortestContact = allStanceFrames.min().map { Int($0) } ?? 0
        let medianContact = measuredStanceFrames
        let taps = WindowedDerivativeFilter.admissibleTaps(medianContact)

        // --- refusals -------------------------------------------------------
        var refusals: [GaitReport.Refusal] = []
        for side in GaitSide.allCases where detected.stance[side].count < minimumContactsPerSide {
            refusals.append(.tooFewContacts(side: side, count: detected.stance[side].count))
        }
        if refusals.isEmpty {
            let strideGapFrames = abs(strideSeconds.left - strideSeconds.right) / dt
            if strideGapFrames > maximumDisagreementFrames {
                refusals.append(.stridePeriodDisagreesBetweenLegs(frames: strideGapFrames))
            }
            let flightGapFrames = abs(modelledFlight - measuredFlight) / dt
            if !(flightGapFrames <= maximumDisagreementFrames) {
                refusals.append(.contactSequenceNotPeriodic(frames: flightGapFrames))
            }
            for side in GaitSide.allCases {
                let v = steadiness.strideVariationPercent[side]
                if !(v <= steadiness.boundPercent) {
                    refusals.append(.strideNotSteady(side: side, percent: v,
                                                     boundPercent: steadiness.boundPercent))
                }
            }
            if !force.describesRunning {
                refusals.append(.notRunning(dutyFactor: force.dutyFactor,
                                            flightToContactRatio: force.flightToContactRatio))
            }
            if measuredStanceFrames != detected.stanceFrameBudget {
                refusals.append(.stanceBudgetInconsistent(assumed: detected.stanceFrameBudget,
                                                          measured: measuredStanceFrames))
            }
            if !(framesPerContact >= 3) {
                refusals.append(.contactTooShortToResolve(framesPerContact: framesPerContact))
            }
            // A contact with a hole is not timed to ±½ a sampling interval, so
            // its duration must not enter a left/right claim. Flagging it was
            // not enough: Monte Carlo at the measured 7.1 % drop rate fabricates
            // up to 19 % of asymmetry through the clock-based duration alone.
            for side in GaitSide.allCases {
                let inside = detected.stance[side].reduce(0) { $0 + $1.droppedSamplesInside }
                let atEdges = detected.stance[side].reduce(0) { $0 + $1.droppedSamplesAtEdges }
                if inside + atEdges > 0 {
                    refusals.append(.droppedSamplesInContact(side: side, inside: inside,
                                                             atEdges: atEdges))
                }
            }
            if medianContact < WindowedDerivativeFilter.minimumSmoothingTaps {
                refusals.append(.contactTooShortForACleanDerivative(
                    medianSamples: medianContact,
                    neededSamples: WindowedDerivativeFilter.minimumSmoothingTaps))
            }
        }

        // --- flags ----------------------------------------------------------
        var flags: [GaitReport.Flag] = []
        if signal.droppedFrameCount > 0 {
            flags.append(.droppedFrames(count: signal.droppedFrameCount,
                                        largestGapInFrames: signal.maximumGapInFrames))
        }
        let clippedCount = detected.edgeClipped.left.count + detected.edgeClipped.right.count
        if clippedCount > 0 {
            flags.append(.edgeClippedRunsExcluded(count: clippedCount))
        }
        let asymmetry = 100 * (contactSeconds.left - contactSeconds.right) / meanContact
        if refusals.isEmpty, !resolution.permitsAsymmetryClaim(ofPercent: asymmetry) {
            flags.append(.asymmetryBelowResolution(measuredPercent: asymmetry,
                                                   resolvablePercent: resolution.resolvableAsymmetryPercent))
        }

        return GaitReport(frameCount: signal.frameCount,
                          sampleInterval: dt,
                          droppedFrameCount: signal.droppedFrameCount,
                          largestGapInFrames: signal.maximumGapInFrames,
                          stridePeriodFrames: detected.stridePeriodFrames,
                          stridePeriodCorrelation: detected.stridePeriodCorrelation,
                          runningSpeedMetersPerSecond: detected.plateauSpeed,
                          plateauScatterMetersPerSecond: detected.plateauScatter,
                          detectionLevelMetersPerSecond: detected.level,
                          stanceFrameBudget: detected.stanceFrameBudget,
                          measuredStanceFrames: measuredStanceFrames,
                          stance: detected.stance,
                          edgeClipped: detected.edgeClipped,
                          contactSeconds: contactSeconds,
                          contactFrames: contactFrames,
                          contactVariationPercent: contactVariation,
                          strideSeconds: strideSeconds,
                          strideVariationPercent: strideVariation,
                          measuredFlightSeconds: measuredFlight,
                          measuredFlightScatterSeconds: flightScatter,
                          modelledFlightSeconds: modelledFlight,
                          force: force,
                          peakVerticalForceInBodyWeights: peaks.peaks,
                          peakVerticalForceIsSharedBetweenLegs: peaks.sharedBetweenLegs,
                          resolution: resolution,
                          steadiness: steadiness,
                          refusals: refusals,
                          flags: flags,
                          shortestContactSamples: shortestContact,
                          medianContactSamples: medianContact,
                          derivativeFilterTaps: taps)
    }
}

/// The larger of two values, ignoring a non-finite one rather than propagating
/// it. `Swift.max` compares with `<`, which is false for every comparison
/// involving NaN, so `Swift.max(.nan, 5)` returns NaN and `Swift.max(5, .nan)`
/// returns NaN too — either way a leg with too few contacts to have a stride
/// period would poison the published resolution.
func largerFinite(_ a: Double, _ b: Double) -> Double {
    switch (a.isFinite, b.isFinite) {
    case (true, true): return Swift.max(a, b)
    case (true, false): return a
    case (false, true): return b
    case (false, false): return .infinity
    }
}
