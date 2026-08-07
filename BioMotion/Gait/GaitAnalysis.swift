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
struct GaitResolution: Equatable {
    let framesPerContact: Double
    let quantisationFloorPercent: Double
    let strideRepeatabilityPercent: Double
    /// `max` of the two. The finest left/right difference this clip may assert.
    let resolvableAsymmetryPercent: Double

    init(framesPerContact: Double, strideRepeatabilityPercent: Double) {
        self.framesPerContact = framesPerContact
        let floor = framesPerContact > 0 ? 100 * 0.5 / framesPerContact : .infinity
        quantisationFloorPercent = floor
        self.strideRepeatabilityPercent = strideRepeatabilityPercent
        resolvableAsymmetryPercent = Swift.max(floor, strideRepeatabilityPercent)
    }

    /// The gate every asymmetry claim has to pass. A difference finer than the
    /// clip's own resolution is refused rather than shown with a caveat.
    func permitsAsymmetryClaim(ofPercent percent: Double) -> Bool {
        percent.isFinite && abs(percent) >= resolvableAsymmetryPercent
    }

    /// The lever, when a claim is refused. Contact duration is a property of the
    /// runner, so the only thing the user can change is the sampling rate.
    func framesPerSecondNeeded(for percent: Double, currentFPS: Double) -> Double {
        guard percent > 0, framesPerContact > 0 else { return .infinity }
        let neededFrames = 100 * 0.5 / percent
        return currentFPS * neededFrames / framesPerContact
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
        case flightTimeDisagreesWithStrideClosure(frames: Double)
        case strideNotSteady(side: GaitSide, percent: Double, boundPercent: Double)
        case notRunning(dutyFactor: Double, flightToContactRatio: Double)
        case stanceBudgetInconsistent(assumed: Int, measured: Int)
        case contactTooShortToResolve(framesPerContact: Double)

        var description: String {
            switch self {
            case .tooFewContacts(let side, let count):
                return "\(side.rawValue): only \(count) complete contacts in this window"
            case .stridePeriodDisagreesBetweenLegs(let frames):
                return String(format: "the two legs report stride periods %.2f frames apart; "
                              + "in steady gait they alternate within one cycle and must agree", frames)
            case .flightTimeDisagreesWithStrideClosure(let frames):
                return String(format: "flight time measured between contacts and flight time implied "
                              + "by closing the stride disagree by %.2f frames", frames)
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
    ///
    /// These two are computed from different things — one from the ordering of
    /// events, one from the stride period and the two contact durations — so
    /// they can disagree, and their disagreement is this stage's falsifier.
    /// Measured: 0.01 frames on `video_012`, 0.01 on `video_015`, 0.90 on
    /// `video_013`.
    let modelledFlightSeconds: Double
    var flightDisagreementFrames: Double {
        abs(modelledFlightSeconds - measuredFlightSeconds) / sampleInterval
    }

    let force: GaitForceModel
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

    /// The longest CENTRED, odd-tap filter window that fits inside the shortest
    /// contact in this clip, so a filter using it never straddles a touchdown or
    /// a toe-off. A `T`-tap centred window spans `(T−1)·dt`, and a contact of `N`
    /// frames spans `(N−1)·dt`, so `T ≤ N`.
    ///
    /// This exists because the engine's Savitzky-Golay filter is 9 taps, which
    /// spans `8·dt = 267 ms` at 30 fps — LONGER than every contact measured
    /// here (100-233 ms). Under that window no stance frame has a neighbourhood
    /// free of a discontinuity (0 of 114 interior frames on `video_012`), and
    /// the same filter's second-derivative gain is 0.49 at the step fundamental
    /// and inverts sign above 7 Hz. Measured answers: 3 taps on `video_012`,
    /// 5 on `video_015`, 1 on `video_013`.
    let filterTapsThatFitOneContact: Int
    /// `8·dt` — what the engine's 9-tap window actually spans on this clip.
    var nineTapFilterSpanSeconds: Double { 8 * sampleInterval }
    var nineTapFilterFitsOneContact: Bool { filterTapsThatFitOneContact >= 9 }
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
            let framesPer = intervals.map { Double($0.frames) }
            allStanceFrames.append(contentsOf: framesPer)
            contactFrames[side] = mean(framesPer)
            contactSeconds[side] = mean(framesPer) * dt
            contactVariation[side] = 100 * coefficientOfVariation(framesPer)
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
        let repeatability = Swift.max(contactVariation.left, contactVariation.right)
        let resolution = GaitResolution(framesPerContact: framesPerContact,
                                        strideRepeatabilityPercent: repeatability)
        let steadiness = GaitSteadiness(strideVariationPercent: strideVariation,
                                        stridePeriodFrames: detected.stridePeriodFrames)

        let measuredStanceFrames = allStanceFrames.isEmpty ? 0 : Int(median(allStanceFrames).rounded())
        let shortestContact = allStanceFrames.min().map { Int($0) } ?? 0
        let taps = shortestContact >= 1 ? (shortestContact % 2 == 1 ? shortestContact : shortestContact - 1) : 0

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
                refusals.append(.flightTimeDisagreesWithStrideClosure(frames: flightGapFrames))
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
                          resolution: resolution,
                          steadiness: steadiness,
                          refusals: refusals,
                          flags: flags,
                          filterTapsThatFitOneContact: taps)
    }
}
