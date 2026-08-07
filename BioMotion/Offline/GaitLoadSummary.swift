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
    /// Stance frames that produced muscle numbers.
    let stanceFrameCount: Int
    /// How many muscles hit the `a ≤ 1` bound anywhere. Where they do, the
    /// "ratios survive a common scale" argument stops being exact.
    let saturatedMuscleCount: Int

    // --- the falsifier, aggregated over the clip -------------------------
    /// Largest `‖ΣF_contact − F_gait‖/(m·g)` over the clip's stance frames.
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
    /// Always false. The gait model supplies a vertical force only; braking and
    /// propulsion along the running direction are not modelled, and this is the
    /// field that says so out loud.
    let horizontalRootAccelerationModelled: Bool

    // --- filter, published because it is a correctness property -----------
    let derivativeFilterTaps: Int
    let derivativeFilterSpanMilliseconds: Double
    let shortestContactMilliseconds: Double

    /// Whether a stated left/right difference may be claimed at all.
    func permits(differencePercent: Double) -> Bool {
        differencePercent.isFinite
            && abs(differencePercent) >= resolvableAsymmetryPercent
            && residualGatePassed
    }

    /// The sentence for one muscle's left/right comparison, or the refusal —
    /// which always names the lever.
    func claim(for load: MuscleLoad) -> String {
        let d = load.differencePercent
        guard d.isFinite else { return "No load measured on one side." }
        if !residualGatePassed {
            return String(format: "Withheld: the model's force disagrees with the body's own "
                          + "inertia by %.2f body weights.", maxForceResidualInBodyWeights)
        }
        if abs(d) < resolvableAsymmetryPercent {
            return String(format: "Even to within what this clip can resolve (±%.0f%%).",
                          resolvableAsymmetryPercent)
        }
        return String(format: "%.0f%% harder on the %@.", abs(d), load.heavierSide)
    }

    /// The frame rate that would resolve `target` percent, given this clip's
    /// contacts. The one actionable thing a user can change.
    func frameRateNeeded(forPercent target: Double) -> Double {
        guard target > 0, framesPerContact > 0 else { return .infinity }
        return framesPerSecond * (100 * 0.5 / target) / framesPerContact
    }

    /// One line stating what this clip resolves and what to do about it.
    var resolutionSentence: String {
        let base = String(format: "This clip resolves left/right to about ±%.0f%% "
                          + "(%.1f frames per contact at %.0f fps).",
                          resolvableAsymmetryPercent, framesPerContact, framesPerSecond)
        guard resolvableAsymmetryPercent > 5,
              quantisationFloorPercent >= strideRepeatabilityPercent else { return base }
        let needed = frameRateNeeded(forPercent: 5)
        guard needed.isFinite, needed > framesPerSecond else { return base }
        return base + String(format: " Filming at %.0f fps would resolve ±5%%.", needed)
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
        var residuals: [Double] = []
        var disagreements = 0
        var saturated = Set<String>()
        var stanceFrames = 0

        for frame in frames {
            guard frame.isGaitStance,
                  let muscle = frame.muscleResult,
                  let outcome = frame.motionState.gaitOutcome else { continue }
            stanceFrames += 1
            residuals.append(outcome.residualInBodyWeights)
            if !outcome.contactDetectorsAgree { disagreements += 1 }

            let onLeft = outcome.contactSide < 0
            for (name, activation) in muscle.activations {
                guard let (base, side) = split(name) else { continue }
                // Only credit a muscle to the leg that is actually on the
                // ground. The swing leg is doing something, but it is not
                // carrying the contact, and mixing the two would make the
                // left/right comparison a comparison of phases.
                guard side == (onLeft ? "l" : "r") else { continue }
                if activation >= 0.999 { saturated.insert(name) }
                if onLeft {
                    leftPeaks[base] = Swift.max(leftPeaks[base] ?? 0, activation)
                    leftFrames[base, default: 0] += 1
                } else {
                    rightPeaks[base] = Swift.max(rightPeaks[base] ?? 0, activation)
                    rightFrames[base, default: 0] += 1
                }
            }
        }

        guard stanceFrames > 0 else { return nil }

        let bilateral = Set(leftPeaks.keys).intersection(rightPeaks.keys)
        let loads = bilateral.map { base in
            MuscleLoad(id: base,
                       displayName: prettyName(base),
                       leftPeak: leftPeaks[base] ?? 0,
                       rightPeak: rightPeaks[base] ?? 0,
                       leftFrames: leftFrames[base] ?? 0,
                       rightFrames: rightFrames[base] ?? 0)
        }
        .filter { $0.peak > 0 }
        .sorted { ($0.peak, $1.id) > ($1.peak, $0.id) }

        let sortedResiduals = residuals.sorted()
        let maxResidual = sortedResiduals.last ?? 0
        let medianResidual = sortedResiduals.isEmpty
            ? 0 : sortedResiduals[sortedResiduals.count / 2]

        let taps = WindowedDerivativeFilter.admissibleTaps(filterTaps)
        let shortest = Double(report.filterTapsThatFitOneContact) * report.sampleInterval

        return GaitLoadSummary(
            ranked: loads,
            resolvableAsymmetryPercent: report.resolution.resolvableAsymmetryPercent,
            quantisationFloorPercent: report.resolution.quantisationFloorPercent,
            strideRepeatabilityPercent: report.resolution.strideRepeatabilityPercent,
            framesPerContact: report.resolution.framesPerContact,
            framesPerSecond: framesPerSecond,
            stanceFrameCount: stanceFrames,
            saturatedMuscleCount: saturated.count,
            maxForceResidualInBodyWeights: maxResidual,
            medianForceResidualInBodyWeights: medianResidual,
            residualGatePassed: maxResidual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
            contactDetectorDisagreements: disagreements,
            horizontalRootAccelerationModelled: false,
            derivativeFilterTaps: taps,
            derivativeFilterSpanMilliseconds: 1000 * Double(taps - 1) * report.sampleInterval,
            shortestContactMilliseconds: 1000 * shortest)
    }

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
