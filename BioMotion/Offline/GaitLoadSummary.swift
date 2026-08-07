import Foundation

/// The RELATIVE view of a running clip — this muscle against that one, left
/// against right, each load normalised — plus the per-clip resolution that says
/// which of those comparisons the clip is actually allowed to make.
///
/// # Why there is no headline newton figure here, and never will be
///
/// The peak ground force this pipeline computes comes from contact and flight
/// TIMING (`GaitForceModel`), and its absolute value carries a criterion-
/// dependent bias of roughly ±28 ms of contact time — about 18 % on `Fmax` at
/// the owner's cadences. Printed on a screen, "2.9 body weights" reads as a
/// measurement. It is not one.
///
/// What survives that error exactly is every RATIO. A peak-force error is a
/// COMMON SCALE over all 520 muscles in one contact, and the muscle QP is
/// linear in the external load while no muscle is saturated, so muscle-to-
/// muscle and left-to-right ratios are untouched by it. (It degrades where
/// activations reach `a ≤ 1`; running peaks do reach that, so `saturatedCount`
/// is published rather than hidden.) That asymmetry between what survives and
/// what does not is the whole reason this type reports percentages and
/// activations and not newtons.
///
/// # Every claim passes the clip's own resolution first
///
/// `GaitResolution` computes, from the measured frames per contact and this
/// runner's own stride-to-stride scatter, the finest left/right difference this
/// clip can distinguish. A difference below it is REFUSED — not shown with a
/// caveat — and the refusal names the frame rate that would resolve it, because
/// capture rate is the only lever that moves the number.
///
/// # The three gates, and the one thing that is NOT gated
///
/// `arePublishable` is the single answer to "may this screen show loads at
/// all", and every consumer — the ranked list, the bars, the 3-D overlay — has
/// to ask it. It used to be true that only the per-muscle SENTENCE went through
/// a gate while the bars rendered unconditionally, so a clip could display a
/// full left/right comparison under a caption reading "loads withheld".
///
/// 1. **`residualGatePassed`** — the omitted inertial term, `‖a_artic‖/g`,
///    measured over the frames where both contact detectors agree. Tests the
///    assumption every ratio rests on: that the timing model's force error is a
///    COMMON SCALE. When it is not, the ratios are not protected.
/// 2. **`contactGatePassed`** — how much of the claimed stance the ID solver's
///    own geometric detector agreed was stance, and how many usable frames each
///    side kept. Two independent signals (foot height above an estimated ground
///    plane, versus pelvis-relative horizontal foot velocity); when they
///    disagree, `solveIDGRF` returns NO ground force and the frame's muscle
///    numbers were solved without the load they are supposed to describe.
///    Those frames are excluded from the peaks rather than averaged in.
/// 3. **`MuscleLoad.isSaturated`, per muscle** — where an activation reaches
///    `a ≤ 1` the QP stops being linear in the external load, and that is
///    exactly and only where a peak-force error stops cancelling out of the
///    ratio. Nothing in this pipeline tests the peak magnitude (see
///    `NimbleEngine.GaitFrameOutcome`), so this is the gate that keeps the
///    untested quantity out of the answer. It is per muscle because saturation
///    is: one clipped adductor says nothing about the soleus.
struct GaitLoadSummary {

    /// One muscle's peak normalised load on each side, each measured during
    /// THAT side's own stance.
    ///
    /// Peak activation, not mean: a correction product is about where the load
    /// concentrates, and a mean over a contact that includes the unloaded tail
    /// dilutes exactly the moment of interest.
    struct MuscleLoad: Identifiable, Equatable {
        /// Solver base name with the side suffix removed, e.g. `glmax1`.
        let id: String
        let displayName: String
        /// 0-1 activation — already a normalised load, which is why it is what
        /// this screen shows.
        let leftPeak: Double
        let rightPeak: Double
        /// How many stance frames each side contributed.
        let leftFrames: Int
        let rightFrames: Int
        /// True when either side's peak reached the QP's `a ≤ 1` bound. A
        /// saturated activation is clipped, so the difference between the two
        /// sides is a difference between two clipped numbers and the linearity
        /// that makes a peak-force error cancel is gone. This muscle's
        /// left/right claim is withheld.
        let isSaturated: Bool

        var peak: Double { Swift.max(leftPeak, rightPeak) }

        /// Left minus right, as a percentage of their mean. Positive = the left
        /// side worked harder.
        var differencePercent: Double {
            let m = 0.5 * (leftPeak + rightPeak)
            guard m > 0 else { return .nan }
            return 100 * (leftPeak - rightPeak) / m
        }

        var heavierSide: String { leftPeak >= rightPeak ? "left" : "right" }
    }

    /// Bilateral muscles, ranked by the heavier side's peak. The ranking IS the
    /// muscle-to-muscle comparison: position 1 is the muscle carrying the most
    /// of this runner's stance load.
    let ranked: [MuscleLoad]
    /// The finest left/right difference this clip may assert, percent.
    let resolvableAsymmetryPercent: Double
    /// Its two components, shown separately because they are different things:
    /// what the sampling grid allows, and what this runner's own strides did.
    let quantisationFloorPercent: Double
    let strideRepeatabilityPercent: Double
    let framesPerContact: Double
    let framesPerSecond: Double
    /// Stance frames that produced muscle numbers AND passed every per-frame
    /// condition — both contact detectors agreeing, and a derivative window
    /// that did not cross a contact edge.
    let stanceFrameCount: Int
    /// Stance frames the plan claimed, before those exclusions.
    let claimedStanceFrameCount: Int
    /// How many muscles hit the `a ≤ 1` bound anywhere. Where they do, the
    /// "ratios survive a common scale" argument stops being exact.
    let saturatedMuscleCount: Int

    // --- the falsifier, aggregated over the clip -------------------------
    /// Largest `‖ΣF_contact − F_gait‖/(m·g)` over the USABLE stance frames.
    ///
    /// Frames where the two contact detectors disagreed are excluded, and
    /// deliberately: `solveIDGRF` returns zero ground force when its geometric
    /// detector sees no foot down, so on those frames the residual is the whole
    /// modelled force (~2-3 BW) and measures the detector disagreement, not
    /// `‖a_artic‖/g`. Leaving them in made a single such frame withhold a whole
    /// clip under a sentence about limb inertia that was not what happened.
    let maxForceResidualInBodyWeights: Double
    let medianForceResidualInBodyWeights: Double
    /// False when the residual exceeded `NimbleEngine.maxGaitForceResidualInBodyWeights`
    /// anywhere: the segment acceleration the timing model omits is then
    /// comparable to the gravitational term it is built from, so its force is
    /// no longer a common rescaling of the contact and the ratios above are not
    /// protected.
    let residualGatePassed: Bool
    /// Stance frames where the ID solver's GEOMETRIC contact detector disagreed
    /// with the KINEMATIC stance detector about which foot was down.
    let contactDetectorDisagreements: Int
    /// Stance frames dropped because their derivative window crossed a contact
    /// edge. Not a failure — an honest consequence of 5-7 samples per contact.
    let framesWithoutACleanDerivativeWindow: Int
    /// Usable frames per side. A comparison needs both.
    let leftStanceFrameCount: Int
    let rightStanceFrameCount: Int
    /// Always false. The gait model supplies a vertical force only; braking and
    /// propulsion along the running direction are not modelled, and this is the
    /// field that says so out loud.
    let horizontalRootAccelerationModelled: Bool

    // --- filter, published because it is a correctness property -----------
    let derivativeFilterTaps: Int
    let derivativeFilterSpanMilliseconds: Double
    let shortestContactMilliseconds: Double
    /// `‖c_acc‖` relative to the 9-tap window the live path uses. The price of
    /// a window short enough to fit inside a contact, and it is per-DOF
    /// independent noise — which does NOT cancel out of a ratio.
    let derivativeNoiseAmplification: Double

    /// Usable stance frames per side, minimum, before a left/right comparison
    /// means anything. Two frames is one number and its confirmation.
    static let minimumUsableStanceFramesPerSide = 2
    /// The fraction of the claimed stance the two independent contact detectors
    /// have to agree on. Below half, the clip's contacts are not what one of the
    /// two detectors thinks they are and which is wrong is not decidable here.
    static let minimumContactAgreementFraction = 0.5

    /// What fraction of the claimed stance frames survived every per-frame
    /// condition.
    var usableStanceFraction: Double {
        claimedStanceFrameCount > 0
            ? Double(stanceFrameCount) / Double(claimedStanceFrameCount) : 0
    }

    /// The contact gate: enough usable frames on both sides, and the two
    /// detectors agreeing on enough of the claimed stance.
    var contactGatePassed: Bool {
        leftStanceFrameCount >= Self.minimumUsableStanceFramesPerSide
            && rightStanceFrameCount >= Self.minimumUsableStanceFramesPerSide
            && agreementFraction >= Self.minimumContactAgreementFraction
    }

    /// Frames both detectors agreed on, over frames the plan claimed.
    var agreementFraction: Double {
        guard claimedStanceFrameCount > 0 else { return 0 }
        return Double(claimedStanceFrameCount - contactDetectorDisagreements)
            / Double(claimedStanceFrameCount)
    }

    /// **The single question every consumer of `ranked` must ask.** Bars,
    /// numbers, ranking and the 3-D overlay are all withheld together when this
    /// is false — they all rest on the same assumptions.
    var arePublishable: Bool { residualGatePassed && contactGatePassed && !ranked.isEmpty }

    /// Why the loads are withheld, naming the measurement AND the lever. Nil
    /// when they are not.
    var withheldReason: String? {
        guard !arePublishable else { return nil }
        if ranked.isEmpty {
            return "No contact produced muscle output on both sides."
        }
        if leftStanceFrameCount < Self.minimumUsableStanceFramesPerSide
            || rightStanceFrameCount < Self.minimumUsableStanceFramesPerSide {
            return "Withheld: only \(leftStanceFrameCount) left and \(rightStanceFrameCount) right "
                 + "stance frames survived (of \(claimedStanceFrameCount) claimed) — "
                 + "\(contactDetectorDisagreements) where the foot's height above the ground "
                 + "disagreed that it was planted, \(framesWithoutACleanDerivativeWindow) with too "
                 + "few frames around them to differentiate. Film more strides, side-on, at a "
                 + "higher frame rate."
        }
        if agreementFraction < Self.minimumContactAgreementFraction {
            return String(format: "Withheld: the foot's height above the ground agreed with the "
                          + "measured contact on only %.0f%% of stance frames (need %.0f%%). The "
                          + "two disagree about which foot is carrying the load. Film side-on, "
                          + "with the whole body and the ground in frame.",
                          100 * agreementFraction, 100 * Self.minimumContactAgreementFraction)
        }
        return String(format: "Withheld: on the frames both contact tests agreed on, the body's own "
                      + "limb acceleration reaches %.2f body weights against a %.2f gate — the "
                      + "timing model's force is not a single scale on this clip, so muscle-to-"
                      + "muscle ratios are not protected. Film a steadier, straighter run.",
                      maxForceResidualInBodyWeights,
                      NimbleEngine.maxGaitForceResidualInBodyWeights)
    }

    /// Whether a stated left/right difference may be claimed at all.
    func permits(differencePercent: Double) -> Bool {
        differencePercent.isFinite
            && abs(differencePercent) >= resolvableAsymmetryPercent
            && arePublishable
    }

    /// The sentence for one muscle's left/right comparison, or the refusal —
    /// which always names the lever.
    func claim(for load: MuscleLoad) -> String {
        let d = load.differencePercent
        guard d.isFinite else { return "No load measured on one side." }
        if let reason = withheldReason { return reason }
        if load.isSaturated {
            return "Withheld: this muscle reached full effort, where the ground-force estimate "
                 + "stops cancelling out of a left/right ratio."
        }
        if abs(d) < resolvableAsymmetryPercent {
            return String(format: "Even to within what this clip can resolve (±%.0f%%).",
                          resolvableAsymmetryPercent)
        }
        return String(format: "%.0f%% harder on the %@.", abs(d), load.heavierSide)
    }

    /// The frame rate that would put the QUANTISATION FLOOR at `target`
    /// percent, given this clip's contacts.
    ///
    /// ⚠️ Not the same as "the rate that would let me claim `target`": the
    /// published resolution is `max(floor, strideRepeatability)` and no camera
    /// moves the second term. `resolutionSentence` checks that before promising.
    func frameRateNeeded(forPercent target: Double) -> Double {
        guard target > 0, framesPerContact > 0 else { return .infinity }
        return framesPerSecond * (100 * 0.5 / target) / framesPerContact
    }

    /// The finest claim any capture rate could support on this runner — the
    /// stride-to-stride variation that survives when the grid stops binding.
    var bestAchievablePercentAtAnyFrameRate: Double { strideRepeatabilityPercent }

    /// One line stating what this clip resolves and what to do about it.
    ///
    /// # The advice has to be deliverable, in two separate senses
    ///
    /// It used to promise ±5 % whenever the floor exceeded the repeatability,
    /// and miss on both counts. On `video_012` (floor 10.1 %, repeatability
    /// 7.2 % under the OLD contact-duration input) it printed "filming at 61 fps
    /// would resolve ±5 %" while `max(4.99, 7.204)` is 7.2 % — missed by 44 %.
    /// And the shipped sampler holds a FRAME budget, so at 120 fps the analysis
    /// window would have collapsed from 4 s to 1 s and the clip would have been
    /// refused for too few contacts. Both are checked here now: the target is
    /// never finer than this runner allows, and the rate is never higher than
    /// `FrameSource` can still analyse a full window at.
    var resolutionSentence: String {
        let base = String(format: "This clip resolves left/right to about ±%.0f%% "
                          + "(%.1f frames per contact at %.0f fps).",
                          resolvableAsymmetryPercent, framesPerContact, framesPerSecond)
        // The finest thing worth promising: never below 5 %, never below what
        // this runner's own strides allow.
        let target = Swift.max(5.0, bestAchievablePercentAtAnyFrameRate)
        guard target.isFinite, resolvableAsymmetryPercent > target else { return base }
        let needed = frameRateNeeded(forPercent: target)
        guard needed.isFinite, needed > framesPerSecond,
              needed <= FrameSource.highestAnalysableFrameRate else {
            return base + " A higher frame rate is the only lever, and this clip is already at "
                 + "what the analysis window can cover."
        }
        return base + String(format: " Filming at %.0f fps would resolve ±%.0f%%.", needed, target)
    }

    /// What the sampling grid cannot see versus what the runner actually varied
    /// — kept apart, because only the first is the camera's fault.
    var resolutionBreakdownSentence: String {
        String(format: "Sampling grid ±%.0f%%; this runner's own stride-to-stride "
               + "variation ±%.0f%%. The claim floor is the larger.",
               quantisationFloorPercent, strideRepeatabilityPercent)
    }

    /// The unmodelled-term disclosure. Shown, not buried.
    var unmodelledTermSentence: String {
        "Vertical load only. Braking and push-off along the running direction are not "
        + "modelled, so fore-aft joint loads are missing a term."
    }

    // MARK: - Construction

    /// Builds the summary from the frames a run produced. `nil` when the clip
    /// has no stance frame carrying muscle output — which is the honest answer
    /// for a clip that was never a run.
    static func make(frames: [OfflineResultStore.FrameResult],
                     report: GaitReport,
                     framesPerSecond: Double,
                     filterTaps: Int) -> GaitLoadSummary? {
        var leftPeaks: [String: Double] = [:]
        var rightPeaks: [String: Double] = [:]
        var leftFrames: [String: Int] = [:]
        var rightFrames: [String: Int] = [:]
        var saturatedBases = Set<String>()
        var residuals: [Double] = []
        var disagreements = 0
        var noCleanWindow = 0
        var saturated = Set<String>()
        var claimedStance = 0
        var usableStance = 0
        var usableLeft = 0
        var usableRight = 0

        for frame in frames {
            guard frame.isGaitStance,
                  let outcome = frame.motionState.gaitOutcome else { continue }
            claimedStance += 1
            if !outcome.contactDetectorsAgree { disagreements += 1 }
            if !outcome.derivativeWindowInsideContact { noCleanWindow += 1 }

            // A frame whose two contact detectors disagree was solved with the
            // WRONG external load — `solveIDGRF` returns no ground force at all
            // when its geometric detector sees no foot down — and a frame whose
            // derivative window crossed a contact edge carries an acceleration
            // fitted across a discontinuity. Neither belongs in a peak, and
            // neither belongs in the residual statistic, where it would be read
            // as limb inertia.
            guard outcome.isUsableForLoadComparison else { continue }
            residuals.append(outcome.residualInBodyWeights)

            guard let muscle = frame.muscleResult else { continue }
            usableStance += 1
            let onLeft = outcome.contactSide < 0
            if onLeft { usableLeft += 1 } else { usableRight += 1 }
            for (name, activation) in muscle.activations {
                guard let (base, side) = split(name) else { continue }
                // Only credit a muscle to the leg that is actually on the
                // ground. The swing leg is doing something, but it is not
                // carrying the contact, and mixing the two would make the
                // left/right comparison a comparison of phases.
                guard side == (onLeft ? "l" : "r") else { continue }
                if activation >= saturationThreshold {
                    saturated.insert(name)
                    saturatedBases.insert(base)
                }
                if onLeft {
                    leftPeaks[base] = Swift.max(leftPeaks[base] ?? 0, activation)
                    leftFrames[base, default: 0] += 1
                } else {
                    rightPeaks[base] = Swift.max(rightPeaks[base] ?? 0, activation)
                    rightFrames[base, default: 0] += 1
                }
            }
        }

        guard claimedStance > 0 else { return nil }

        let bilateral = Set(leftPeaks.keys).intersection(rightPeaks.keys)
        let loads = bilateral.map { base in
            MuscleLoad(id: base,
                       displayName: prettyName(base),
                       leftPeak: leftPeaks[base] ?? 0,
                       rightPeak: rightPeaks[base] ?? 0,
                       leftFrames: leftFrames[base] ?? 0,
                       rightFrames: rightFrames[base] ?? 0,
                       isSaturated: saturatedBases.contains(base))
        }
        .filter { $0.peak > 0 }
        .sorted { ($0.peak, $1.id) > ($1.peak, $0.id) }

        let sortedResiduals = residuals.sorted()
        let maxResidual = sortedResiduals.last ?? 0
        let medianResidual = sortedResiduals.isEmpty
            ? 0 : sortedResiduals[sortedResiduals.count / 2]

        let taps = WindowedDerivativeFilter.admissibleTaps(filterTaps)
        let shortest = Double(report.shortestContactSamples) * report.sampleInterval

        return GaitLoadSummary(
            ranked: loads,
            resolvableAsymmetryPercent: report.resolution.resolvableAsymmetryPercent,
            quantisationFloorPercent: report.resolution.quantisationFloorPercent,
            strideRepeatabilityPercent: report.resolution.strideRepeatabilityPercent,
            framesPerContact: report.resolution.framesPerContact,
            framesPerSecond: framesPerSecond,
            stanceFrameCount: usableStance,
            claimedStanceFrameCount: claimedStance,
            saturatedMuscleCount: saturated.count,
            maxForceResidualInBodyWeights: maxResidual,
            medianForceResidualInBodyWeights: medianResidual,
            residualGatePassed: maxResidual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
            contactDetectorDisagreements: disagreements,
            framesWithoutACleanDerivativeWindow: noCleanWindow,
            leftStanceFrameCount: usableLeft,
            rightStanceFrameCount: usableRight,
            horizontalRootAccelerationModelled: false,
            derivativeFilterTaps: taps,
            derivativeFilterSpanMilliseconds: 1000 * Double(taps - 1) * report.sampleInterval,
            shortestContactMilliseconds: 1000 * shortest,
            derivativeNoiseAmplification: WindowedDerivativeFilter
                .accelerationNoiseAmplification(taps: taps))
    }

    /// The activation at which the QP's `a ≤ 1` bound is treated as reached.
    /// OSQP stops on a tolerance, so an exactly-1.0 test would miss a clipped
    /// muscle by a few units in the last place.
    static let saturationThreshold = 0.999

    /// `"glmax1_r"` → `("glmax1", "r")`. Nil for a muscle with no side, which
    /// is every trunk and spine muscle in `FullBody.osim` — those cannot enter
    /// a left/right comparison and are excluded rather than paired with
    /// themselves.
    static func split(_ name: String) -> (base: String, side: String)? {
        guard name.count > 2 else { return nil }
        let suffix = String(name.suffix(2))
        guard suffix == "_l" || suffix == "_r" else { return nil }
        return (String(name.dropLast(2)), String(suffix.dropFirst()))
    }

    /// Names for the muscles a runner would recognise. Anything not here keeps
    /// the model's own name — inventing a friendly label for all 260 pairs
    /// would be guessing at anatomy the table does not carry.
    static let displayNames: [String: String] = [
        "glmax1": "Glute max (upper)", "glmax2": "Glute max (mid)", "glmax3": "Glute max (lower)",
        "glmed1": "Glute med (front)", "glmed2": "Glute med (mid)", "glmed3": "Glute med (back)",
        "glmin1": "Glute min (front)", "glmin2": "Glute min (mid)", "glmin3": "Glute min (back)",
        "recfem": "Rectus femoris",
        "vasmed": "Vastus medialis", "vaslat": "Vastus lateralis", "vaslat140": "Vastus lateralis",
        "vasint": "Vastus intermedius",
        "bflh": "Hamstring (biceps long)", "bflh140": "Hamstring (biceps long)",
        "bfsh": "Hamstring (biceps short)",
        "semimem": "Hamstring (semimembranosus)", "semiten": "Hamstring (semitendinosus)",
        "gasmed": "Calf (medial gastroc)", "gaslat": "Calf (lateral gastroc)",
        "gaslat140": "Calf (lateral gastroc)",
        "soleus": "Soleus",
        "tibant": "Tibialis anterior", "tibpost": "Tibialis posterior",
        "perlong": "Peroneus longus", "perbrev": "Peroneus brevis",
        "psoas": "Psoas", "iliacus": "Iliacus",
        "addlong": "Adductor longus", "addbrev": "Adductor brevis",
        "addmagDist": "Adductor magnus (distal)", "addmagIsch": "Adductor magnus (ischial)",
        "addmagMid": "Adductor magnus (mid)", "addmagProx": "Adductor magnus (proximal)",
        "tfl": "Tensor fasciae latae", "sart": "Sartorius", "grac": "Gracilis",
        "edl": "Extensor digitorum longus", "ehl": "Extensor hallucis longus",
        "fdl": "Flexor digitorum longus", "fhl": "Flexor hallucis longus",
        "piri": "Piriformis",
    ]

    static func prettyName(_ base: String) -> String {
        displayNames[base] ?? base
    }
}
