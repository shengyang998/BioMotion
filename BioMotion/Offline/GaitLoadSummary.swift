import Foundation

/// The RELATIVE view of a running clip — **left against right, one muscle at a
/// time** — plus the per-clip resolution that says which of those comparisons
/// the clip is actually allowed to make.
///
/// # What this type claims, and the one thing it deliberately no longer claims
///
/// It publishes, per muscle, the LEFT/RIGHT comparison. It does NOT publish a
/// muscle-to-muscle ordering, and `muscles` is not sorted by load.
///
/// `MomentArmComputer` logs at load that 76 `PathWrap` references on 66 of
/// `FullBody.osim`'s 520 muscles are not modelled: those paths take a straight
/// line where the real tendon wraps around bone, so their moment arm `r` is
/// wrong, and `F = τ/r` makes the muscle's force — and hence its activation —
/// wrong by a factor of its own. `musclesWithUnmodelledPaths` is that list, and
/// it contains glmax1/2/3, recfem, the vasti, gasmed, the hamstrings, psoas,
/// iliacus, the adductors and grac: essentially every muscle a runner would
/// recognise. Running is a flexed-pose activity, which is where the warning says
/// the error is worst and where the sign can flip.
///
/// **The error is a per-muscle SCALE, so it cancels in a left/right ratio and
/// does not cancel between two muscles.** Both halves of that are measured, on
/// the shipping QP, in `MomentArmErrorCancellationTests`: perturbing one
/// muscle's moment arm on BOTH sides moves no muscle's `differencePercent` by
/// more than a rounding error while moving the cross-muscle quantity by tens of
/// percent and reordering the list. So:
///
/// * the left/right sentence stays, and it is the product;
/// * the ranking is gone. The list is ordered by which left/right comparison
///   this clip resolved best, which is a property of the CLAIMS and not a claim
///   about which muscle worked hardest.
///
/// The same test isolates what the surviving claim depends on: the error must be
/// the same factor on both sides. A one-sided perturbation moves the left/right
/// figure by more than 10 percentage points. The model is bilaterally symmetric
/// and each side is sampled at its own mid-contact, so this holds by
/// construction — but it is an assumption, it is stated on screen, and this is
/// the number it is worth.
///
/// # Why there is no headline newton figure here, and never will be
///
/// The peak ground force this pipeline computes comes from contact and flight
/// TIMING (`GaitForceModel`), and its absolute value carries a criterion-
/// dependent bias of roughly ±28 ms of contact time — about 18 % on `Fmax` at
/// the owner's cadences. Printed on a screen, "2.9 body weights" reads as a
/// measurement. It is not one.
///
/// What survives that error exactly is every LEFT/RIGHT ratio. A peak-force
/// error is a COMMON SCALE over all 520 muscles in one contact, and the muscle
/// QP is linear in the external load while no muscle is saturated. (It degrades
/// where activations reach `a ≤ 1`; running peaks do reach that, so
/// `isSaturated` withholds per muscle.)
///
/// # Every claim passes three floors, not one
///
/// `claimFloorPercent(for:)` is what a stated left/right difference has to
/// clear, and it is built from three separate things:
///
/// 1. **What the clip's sampling grid and this runner's strides allow** —
///    `GaitResolution.resolvableAsymmetryPercent`. Timing only.
/// 2. **What this muscle's own step-to-step scatter allows** —
///    `MuscleLoad.samplingUncertaintyPercent`, the 95 % half-width of
///    `differencePercent` computed from the per-contact samples themselves.
///    This term did not exist, and its absence was measurable: at the scatter
///    `GaitLoadStatisticTests` uses, a perfectly symmetric runner produced a
///    per-clip standard deviation of 9.47 % against a 10.145 % publication
///    floor, i.e. roughly one in four muscles reading a false finding. A floor
///    derived from frames per contact and the stride period cannot see that,
///    because activation scatter is not one of its inputs.
/// 3. **The contact-time difference the force model injects** —
///    `contactTimeContributionPercent`. Where the clip resolves the left/right
///    contact difference, `Fmax_side = (π/2)(1 + tf/tc_side)` differs between
///    the legs, the QP is linear in the external load, and that difference
///    lands inside every muscle's left/right number as a common additive term.
///    The floor is widened by it and the screen prints its size, so a user can
///    subtract it instead of being told only that it is in there somewhere.
///
/// # The clip-level gates
///
/// `arePublishable` is the single answer to "may this screen show loads at
/// all", and every consumer — the list, the bars, the 3-D overlay — has to ask
/// it.
///
/// 1. **`residualGatePassed`** — the VERTICAL disagreement between the timing
///    model's force and the one inverse dynamics solved,
///    `|ΣF_y − F_gait|/(m·g)`, measured over the frames where both contact
///    detectors agree. It is NOT `‖a_artic‖/g`: the fore-aft components exist in
///    the bridge's output and are discarded, so the axis where the known
///    0.2-0.35 BW error lives is not examined by this gate at all. And it is
///    false when no frame reached it, because a check that measured nothing has
///    not passed.
/// 2. **`contactGatePassed`** — enough usable frames AND at least
///    `minimumContactsPerSide` contacts on each side, plus enough agreement
///    between the two independent contact detectors. The contact requirement is
///    not a taste: with one contact on a side there is no second sample to
///    measure that side's step-to-step scatter from, so floor 2 above cannot be
///    computed at all and the claim is uncertifiable rather than merely noisy.
/// 3. **`MuscleLoad.isSaturated`, per muscle** — where an activation reaches
///    the QP's `a ≤ 1` bound the linearity that makes a force error cancel is
///    gone. The threshold comes from `MuscleSolver`'s own OSQP tolerances, not
///    from a hand-picked constant: OSQP stops on `eps_abs + eps_rel·max‖z‖` and
///    this solver accepts `OSQP_SOLVED_INACCURATE` (ten times looser) with
///    polishing off, so a genuinely clipped activation can come back at 0.98.
///    A 0.999 test — which is what shipped — missed every one of them.
struct GaitLoadSummary {

    /// One muscle's normalised load on each side, each measured during THAT
    /// side's own stance.
    ///
    /// # Why this is NOT the maximum over the side's frames
    ///
    /// It used to be `max` over every usable stance frame of that side, and that
    /// is the one statistic this product must not use, because the number of
    /// frames each side contributes is UNEQUAL and the inequality is caused by
    /// the very asymmetry being measured.
    ///
    /// `E[max of n]` grows with `n` — for frame-to-frame activation scatter `σ`
    /// (amplified 4.69× by the 5-tap derivative window, and independent per DOF
    /// so it does NOT cancel out of a ratio) the gap between a 12-frame side and
    /// a 6-frame side is `σ·(√(2 ln 12) − √(2 ln 6)) = 0.34σ` of pure statistical
    /// left-high asymmetry. And 12-vs-6 is not a contrived split: with `taps = 5`
    /// a contact of `n` samples contributes exactly `n − 4` frames with a clean
    /// derivative window, so a leg whose contacts run ONE frame longer
    /// contributes TWICE as many.
    ///
    /// # What replaced it
    ///
    /// **Each contact contributes exactly one sample — the middle of its own
    /// usable frames — and the side's load is the MEAN of those samples.**
    ///
    /// * The within-contact count is fixed at one, so no extreme-value bias can
    ///   enter from a side having longer contacts.
    /// * Across contacts the statistic is a MEAN, whose expectation is exactly
    ///   the population mean for ANY distribution and ANY sample count.
    /// * The middle of the contact is where the modelled half-sine force peaks.
    ///
    /// **The residual count-dependence, stated.** The middle sample of a contact
    /// of `n` samples sits at a modelled force of `sin(π·φ)` with `φ` the
    /// mid-most usable phase: 1.000 for odd `n`, 0.966 for even `n` — 3.4 % of
    /// force scale from parity alone, bounded, and asserted in
    /// `GaitLoadStatisticTests`.
    ///
    /// **And what an unbiased mean still does not give you.** Zero bias is not
    /// zero uncertainty: a mean of `n` per-contact samples has a standard error,
    /// the difference of two of them has a larger one, and until
    /// `samplingUncertaintyPercent` existed nothing on the claim path knew that.
    struct MuscleLoad: Identifiable, Equatable {
        /// Solver base name with the side suffix removed, e.g. `glmax1`.
        let id: String
        let displayName: String
        /// 0-1 activation at mid-contact, averaged over that side's contacts.
        ///
        /// ⚠️ **Comparable to the other side's number and to nothing else.** Its
        /// scale carries this muscle's own moment-arm error; see the type doc.
        let leftLoad: Double
        let rightLoad: Double
        /// How many CONTACTS contributed a sample to each side.
        let leftContacts: Int
        let rightContacts: Int
        /// True when this muscle reached the QP's `a ≤ 1` bound on ANY usable
        /// frame of either side, within the solver's own tolerance. A saturated
        /// activation is clipped, so the difference between the two sides is a
        /// difference between two clipped numbers and the linearity that makes a
        /// force error cancel is gone. This muscle's left/right claim is
        /// withheld.
        let isSaturated: Bool
        /// **True when this muscle sat on the QP's LOWER bound — resting tone,
        /// `a ≥ aMin` — on any usable frame of either side.**
        ///
        /// The same argument as `isSaturated`, at the other end of the box. The
        /// linearity that makes a force error or a moment-arm error cancel holds
        /// only while no bound is active, and `aMin = 0.02` is a bound. It is
        /// reached in practice and not rarely: 520 muscles are redundant for
        /// ~160 coordinates, so most of them are pushed to the floor, and a
        /// muscle whose modelled path has the WRONG SIGN is pushed there too —
        /// `MomentArmErrorCancellationTests` measures a sign-flipped muscle
        /// landing on the floor on both sides and reading exactly 0 % left/right,
        /// which is a lost finding presented as an even one.
        let isAtActivationFloor: Bool
        /// **The 95 % half-width of `differencePercent` implied by this muscle's
        /// own contact-to-contact scatter**, in percent.
        ///
        /// `SE = √(s_L²/n_L + s_R²/n_R)` over the per-contact samples, scaled to
        /// the same denominator `differencePercent` uses and multiplied by the
        /// two-sided 95 % Student-t factor for `min(n_L, n_R) − 1` degrees of
        /// freedom — conservative, because at five or six contacts a normal
        /// multiplier understates the interval by a third.
        ///
        /// Infinite when either side has fewer than two contacts: with one
        /// sample there is no scatter estimate, and an unmeasured uncertainty is
        /// not a small one.
        let samplingUncertaintyPercent: Double
        /// False when this muscle carries `PathWrap` geometry the moment-arm
        /// computer does not model, so its activation is on an unknown scale of
        /// its own. It does NOT withhold the left/right claim — that is the
        /// whole finding — but it is shown per row, because the number beside it
        /// must not be read against the row above.
        let pathIsModelled: Bool

        /// Left minus right, as a percentage of their mean. Positive = the left
        /// side worked harder. **The deliverable.**
        var differencePercent: Double {
            let m = 0.5 * (leftLoad + rightLoad)
            guard m > 0 else { return .nan }
            return 100 * (leftLoad - rightLoad) / m
        }

        var heavierSide: String { leftLoad >= rightLoad ? "left" : "right" }

        /// The heavier side's load, purely so a row's two bars can be drawn to
        /// their own scale. **Never a ranking key** — see the type doc.
        var withinMuscleScale: Double { Swift.max(leftLoad, rightLoad) }
    }

    /// Bilateral muscles, **ordered by how well this clip resolved each one's
    /// left/right difference** — publishable claims first, then by the size of
    /// the difference relative to its own floor. This is an ordering of CLAIMS,
    /// not of muscles: position 1 is the comparison this clip is most sure
    /// about, not the muscle carrying the most load.
    ///
    /// The property was called `ranked` and was sorted by `max(left, right)`.
    /// That sort had two independent defects on top of the moment-arm one: the
    /// key is clipped at `a ≤ 1`, so a clip where twelve muscles saturate sorted
    /// them alphabetically by id; and the bars drawn from it were full length
    /// under a caption saying the comparison was withheld.
    private(set) var muscles: [MuscleLoad]
    /// The finest left/right difference this clip's TIMING may assert, percent.
    /// One of the three inputs to `claimFloorPercent(for:)`.
    let resolvableAsymmetryPercent: Double
    /// Its two components, shown separately because they are different things:
    /// what the sampling grid allows, and what this runner's own strides did.
    let quantisationFloorPercent: Double
    /// `max(measured scatter, what this clip could have distinguished)`.
    let strideRepeatabilityPercent: Double
    /// The raw CV, kept beside the published figure so the floor is visible
    /// rather than silently applied.
    let measuredStrideRepeatabilityPercent: Double
    /// One sampling interval as a fraction of the stride period.
    let strideRepeatabilityBoundPercent: Double
    /// True when both legs' `Fmax` was closed on the MEAN contact time because
    /// the clip could not resolve the difference between them.
    let peakForceIsSharedBetweenLegs: Bool
    /// **How much of every muscle's left/right difference is the contact-time
    /// difference re-expressed as force**, in percent, signed the same way as
    /// `differencePercent`.
    ///
    /// `Fmax_side = (π/2)(1 + tf/tc_side)` and the QP is linear in the external
    /// load, so for identical left/right activations the displayed difference is
    /// exactly this number. Zero when the peaks are shared. It is added to every
    /// claim floor and printed, because "part of the difference in these bars is
    /// that contact-time difference" without a size is not something a user can
    /// act on — it can be all of it.
    let contactTimeContributionPercent: Double
    let framesPerContact: Double
    let framesPerSecond: Double
    /// Stance frames that produced muscle numbers AND passed every per-frame
    /// condition.
    let stanceFrameCount: Int
    /// Stance frames the plan claimed, before those exclusions.
    let claimedStanceFrameCount: Int
    /// How many muscles hit the `a ≤ 1` bound anywhere.
    let saturatedMuscleCount: Int
    /// How many muscles sat on the `a ≥ aMin` bound anywhere. Same argument,
    /// other end of the box — see `MuscleLoad.isAtActivationFloor`.
    let flooredMuscleCount: Int

    // --- the falsifier, aggregated over the clip -------------------------
    /// Largest `|ΣF_contact,y − F_gait|/(m·g)` over the USABLE stance frames.
    ///
    /// # ⚠️ VERTICAL ONLY, and the name says so because the label used not to
    ///
    /// `NimbleEngine.GaitFrameOutcome.residualInBodyWeights` is built from
    /// `leftFootForce.y + rightFootForce.y` against the modelled VERTICAL force.
    /// The bridge publishes `[fx, fy, fz]` for each foot and the other two
    /// components are discarded. See `unmodelledTermSentence` for what is
    /// happening on the axis this number does not cover — it is not "nothing".
    let maxVerticalForceResidualInBodyWeights: Double
    let medianVerticalForceResidualInBodyWeights: Double
    /// How many stance frames the residual statistic was computed from. Zero
    /// means it measured NOTHING — see `residualWasMeasured`.
    let residualFrameCount: Int
    /// False when the residual exceeded `NimbleEngine.maxGaitForceResidualInBodyWeights`
    /// anywhere, and **also false when no frame was measured at all**.
    let residualGatePassed: Bool
    /// Stance frames where the ID solver's GEOMETRIC contact detector disagreed
    /// with the KINEMATIC stance detector about which foot was down.
    let contactDetectorDisagreements: Int
    /// The subset of those where the solver put ground force under BOTH feet.
    ///
    /// Counted separately because the two have different causes and different
    /// levers, and collapsing them made the refusal sentence describe the wrong
    /// one: "the foot's height disagreed that it was planted" is false for a
    /// double contact — the height agreed it was planted, and also flagged the
    /// other foot. `NimbleEngine` already records the distinction
    /// ("'no foot down' points at the ground-height estimate, 'both feet down'
    /// points at the 6 cm contact threshold against the swing foot's
    /// clearance"); it just never reached the clip level.
    let solverSawDoubleContactCount: Int
    /// Stance frames dropped because their derivative window crossed a contact
    /// edge. Not a failure — an honest consequence of 5-7 samples per contact.
    let framesWithoutACleanDerivativeWindow: Int
    /// Usable frames per side. A comparison needs both.
    let leftStanceFrameCount: Int
    let rightStanceFrameCount: Int
    /// Contacts per side that contributed a sample to the load statistic.
    let leftContactCount: Int
    let rightContactCount: Int
    /// Always false. The gait model supplies a vertical force only.
    let horizontalRootAccelerationModelled: Bool

    // --- filter, published because it is a correctness property -----------
    let derivativeFilterTaps: Int
    let derivativeFilterSpanMilliseconds: Double
    let shortestContactMilliseconds: Double
    /// `‖c_acc‖` relative to the 9-tap window the live path uses.
    let derivativeNoiseAmplification: Double

    /// Usable stance frames per side, minimum, before a left/right comparison
    /// means anything. Two frames is one number and its confirmation.
    static let minimumUsableStanceFramesPerSide = 2
    /// **Contacts per side, minimum.** Frames are not contacts: a clip where
    /// detector disagreements wiped out five of one leg's six contacts but left
    /// one with three clean frames used to pass a frame-count gate with
    /// `leftContactCount == 1` against `rightContactCount == 6`, and the two
    /// sides' means then carried `σ` and `σ/√6`. Two is the smallest number that
    /// lets a side's own scatter be estimated at all; below it
    /// `samplingUncertaintyPercent` is infinite by construction.
    static let minimumContactsPerSide = 2
    /// The fraction of the claimed stance the two independent contact detectors
    /// have to agree on.
    static let minimumContactAgreementFraction = 0.5
    /// How many muscles the panel shows. Published here because the per-muscle
    /// interval is a 95 % one, so showing this many makes roughly one in twenty
    /// of them read a difference by chance — a fact the screen states rather
    /// than leaves to the reader.
    static let displayedMuscleCount = 8

    /// What fraction of the claimed stance frames survived every per-frame
    /// condition.
    var usableStanceFraction: Double {
        claimedStanceFrameCount > 0
            ? Double(stanceFrameCount) / Double(claimedStanceFrameCount) : 0
    }

    /// The contact gate: enough usable frames and enough CONTACTS on both
    /// sides, and the two detectors agreeing on enough of the claimed stance.
    var contactGatePassed: Bool {
        leftStanceFrameCount >= Self.minimumUsableStanceFramesPerSide
            && rightStanceFrameCount >= Self.minimumUsableStanceFramesPerSide
            && leftContactCount >= Self.minimumContactsPerSide
            && rightContactCount >= Self.minimumContactsPerSide
            && agreementFraction >= Self.minimumContactAgreementFraction
    }

    /// Whether the vertical falsifier examined any frame at all.
    var residualWasMeasured: Bool { residualFrameCount > 0 }

    /// Frames both detectors agreed on, over frames the plan claimed.
    var agreementFraction: Double {
        guard claimedStanceFrameCount > 0 else { return 0 }
        return Double(claimedStanceFrameCount - contactDetectorDisagreements)
            / Double(claimedStanceFrameCount)
    }

    /// **The single question every consumer of `muscles` must ask.** Bars,
    /// numbers, ordering and the 3-D overlay are all withheld together when this
    /// is false — they all rest on the same assumptions.
    var arePublishable: Bool { residualGatePassed && contactGatePassed && !muscles.isEmpty }

    /// Why the loads are withheld, naming the measurement AND the lever. Nil
    /// when they are not.
    var withheldReason: String? {
        guard !arePublishable else { return nil }
        if muscles.isEmpty {
            return "No contact produced muscle output on both sides."
        }
        if leftStanceFrameCount < Self.minimumUsableStanceFramesPerSide
            || rightStanceFrameCount < Self.minimumUsableStanceFramesPerSide {
            return "Withheld: only \(leftStanceFrameCount) left and \(rightStanceFrameCount) right "
                 + "stance frames survived (of \(claimedStanceFrameCount) claimed) — "
                 + "\(contactDetectorDisagreements) where the two contact tests disagreed, "
                 + "\(framesWithoutACleanDerivativeWindow) with too few frames around them to "
                 + "differentiate. Film more strides, side-on, at a higher frame rate."
        }
        if leftContactCount < Self.minimumContactsPerSide
            || rightContactCount < Self.minimumContactsPerSide {
            return "Withheld: \(leftContactCount) left and \(rightContactCount) right foot "
                 + "contacts survived, and \(Self.minimumContactsPerSide) on each side are the "
                 + "minimum — with one there is no way to measure how much this runner varies "
                 + "from step to step, so a left/right difference cannot be told from that "
                 + "variation. Film a longer run."
        }
        if agreementFraction < Self.minimumContactAgreementFraction {
            return contactDisagreementRefusal
        }
        // There is deliberately no `!residualWasMeasured` branch here: a residual
        // is appended for EVERY usable frame, so an empty residual set means no
        // usable frame, which means `muscles` is empty and the first branch above
        // already fired.
        return String(format: "Withheld: on the frames both contact tests agreed on, the vertical "
                      + "disagreement between the timing model and inverse dynamics reaches %.2f "
                      + "body weights against a %.2f gate — the timing model's force is not a "
                      + "single scale on this clip, so its left/right ratios are not protected. "
                      + "Film a steadier, straighter run.",
                      maxVerticalForceResidualInBodyWeights,
                      NimbleEngine.maxGaitForceResidualInBodyWeights)
    }

    /// The contact-disagreement refusal, split by WHICH disagreement dominates,
    /// because the two have different causes and only one of them has a lever
    /// the user can pull.
    ///
    /// Offering "film side-on with the ground in frame" for a double contact is
    /// advice that cannot work: the ground was never the problem. A double
    /// contact means the swing foot passed inside the solver's 6 cm contact
    /// band — a threshold against a pelvis-pinned stream whose own bounce is
    /// comparable to it — and no re-filming moves that.
    private var contactDisagreementRefusal: String {
        let disagreementPercent = 100 * (1 - agreementFraction)
        let doublePercent = claimedStanceFrameCount > 0
            ? 100 * Double(solverSawDoubleContactCount) / Double(claimedStanceFrameCount) : 0
        let head = String(format: "Withheld: the two contact tests disagreed on %.0f%% of stance "
                          + "frames and had to agree on %.0f%%. ",
                          disagreementPercent,
                          100 * (1 - Self.minimumContactAgreementFraction))
        if solverSawDoubleContactCount * 2 >= contactDetectorDisagreements {
            return head + String(format: "On %.0f%% of them the solver put ground force under "
                                 + "BOTH feet, so the load was split between a planted foot and a "
                                 + "swing foot passing within its 6 cm contact band. That is a "
                                 + "limit of how this app decides a foot is down, not of your "
                                 + "clip — re-filming will not move it.", doublePercent)
        }
        return head + "On most of them the solver saw NO foot down, so its estimate of where the "
             + "ground is did not match the measured contact. Film side-on, with the whole body "
             + "and the ground in frame."
    }

    /// **The floor THIS muscle's left/right difference has to clear.**
    ///
    /// `max(what the clip's timing resolves, what this muscle's own step-to-step
    /// scatter resolves) + |the contact-time term the force model injects|`.
    /// The first two are competing lower bounds on a distinguishable difference;
    /// the third is a known additive contamination that has to be taken off
    /// before what is left can be attributed to the muscle.
    func claimFloorPercent(for load: MuscleLoad) -> Double {
        Swift.max(resolvableAsymmetryPercent, load.samplingUncertaintyPercent)
            + abs(contactTimeContributionPercent)
    }

    /// Whether this muscle's stated left/right difference may be claimed.
    ///
    /// It takes the MUSCLE, not a bare percentage: the floor depends on that
    /// muscle's own contact-to-contact scatter, and a signature that accepts a
    /// lone number cannot ask about it. The old one could not, which is exactly
    /// how a claim came to be gated on timing quantisation alone.
    func permits(_ load: MuscleLoad) -> Bool {
        let d = load.differencePercent
        return d.isFinite
            && !load.isSaturated
            && !load.isAtActivationFloor
            && abs(d) >= claimFloorPercent(for: load)
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
        if load.isAtActivationFloor {
            return "Withheld: this muscle sat at the solver's resting-tone floor, where the same "
                 + "cancellation stops holding — the number below the floor is not recoverable, "
                 + "so a left/right ratio taken across it is not either."
        }
        let floor = claimFloorPercent(for: load)
        guard floor.isFinite else {
            return "Withheld: \(load.leftContacts) left and \(load.rightContacts) right contacts "
                 + "carried this muscle, and two on each side are the minimum for measuring how "
                 + "much it varies from step to step."
        }
        if abs(d) < floor {
            return String(format: "Even to within what this clip and this muscle's own "
                          + "step-to-step scatter can resolve (±%.0f%%).", floor)
        }
        return String(format: "%.0f%% harder on the %@ (needs ±%.0f%%).", abs(d),
                      load.heavierSide, floor)
    }

    /// The frame rate that would put the QUANTISATION FLOOR at `target`
    /// percent, given this clip's contacts.
    ///
    /// ⚠️ Not the same as "the rate that would let me claim `target`": the
    /// published resolution is `max(floor, strideRepeatability)` and no camera
    /// moves the second term, nor the per-muscle scatter term.
    func frameRateNeeded(forPercent target: Double) -> Double {
        guard target > 0, framesPerContact > 0 else { return .infinity }
        return framesPerSecond * (100 * 0.5 / target) / framesPerContact
    }

    /// The finest claim any capture rate could support on this runner's TIMING.
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
    /// refused for too few contacts. Both are checked here now.
    var resolutionSentence: String {
        let base = String(format: "This clip's timing resolves left/right to about ±%.0f%% "
                          + "(%.1f frames per contact at %.0f fps). Each muscle also has to "
                          + "clear its own step-to-step scatter, shown with its claim.",
                          resolvableAsymmetryPercent, framesPerContact, framesPerSecond)
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
        let grid = String(format: "Sampling grid ±%.0f%%; ", quantisationFloorPercent)
        if measuredStrideRepeatabilityPercent < strideRepeatabilityBoundPercent {
            return grid + String(format: "this runner's strides varied by less than the ±%.1f%% "
                                 + "this clip could have seen (measured ±%.1f%% on whole-frame "
                                 + "touchdowns), so ±%.1f%% is what may be assumed. The claim "
                                 + "floor is the larger of the two.",
                                 strideRepeatabilityBoundPercent,
                                 measuredStrideRepeatabilityPercent,
                                 strideRepeatabilityPercent)
        }
        return grid + String(format: "this runner's own stride-to-stride variation ±%.0f%%. "
                             + "The claim floor is the larger.", strideRepeatabilityPercent)
    }

    /// **What is NOT measured on the fore-aft axis — which is not the same as
    /// what is missing from it.**
    ///
    /// This sentence used to say braking and push-off were "not modelled, so
    /// fore-aft joint loads are missing a term". That is false in the direction
    /// that flatters the output. `getMultipleContactInverseDynamicsNearCoP`
    /// solves full 6-D wrenches per foot subject to Newton-Euler, so it ASSIGNS
    /// a fore-aft ground force on every stance frame. What it assigns is
    /// whatever makes the horizontal CoM acceleration match the pose stream —
    /// and `MHRRetarget` pins the pelvis, so that acceleration is ~0 whether
    /// `pelvis_tx`/`pelvis_tz` are forced to zero or merely left alone. The
    /// distinction the engine's comment draws between those two is numerically
    /// empty on this pose source. STATUS sizes the resulting error at 0.2-0.35
    /// BW, larger than the worst vertical residual the same screen reports as
    /// passing, and it is phase-dependent — braking early, propulsion late — so
    /// unlike a common scale it does not cancel out of a comparison.
    ///
    /// It DOES still cancel out of a left/right comparison of one muscle, for
    /// the same reason a moment-arm error does: both legs are sampled at their
    /// own mid-contact through the same fabrication.
    var unmodelledTermSentence: String {
        "Fore-aft braking and push-off is not measured — and it is not left out either. Inverse "
        + "dynamics assigns whatever horizontal ground force makes the body match a pelvis the "
        + "pose model holds still, so every joint moment here contains a fabricated fore-aft "
        + "term of roughly 0.2-0.35 body weights. It changes through the contact, so it does not "
        + "cancel between two different muscles. It does cancel between your left and right, "
        + "which is why that is the only comparison shown."
    }

    /// The falsifier line, named for the axis it actually measures.
    var verticalFalsifierSentence: String {
        guard residualWasMeasured else {
            return "Vertical disagreement between the timing model and inverse dynamics: NOT "
                 + "MEASURED — no stance frame cleared both contact tests with a clean derivative "
                 + "window, so this check ran on nothing."
        }
        return String(format: "Vertical disagreement between the timing model and inverse "
                      + "dynamics: %.2f BW typical, %.2f BW worst (gate %.2f) — %@. Vertical "
                      + "axis only: the fore-aft term named above is present and fabricated, and "
                      + "neither check here can see it.",
                      medianVerticalForceResidualInBodyWeights,
                      maxVerticalForceResidualInBodyWeights,
                      NimbleEngine.maxGaitForceResidualInBodyWeights,
                      residualGatePassed ? "passed" : "FAILED")
    }

    /// Which regime the per-leg peak force is in, **with the size of what it
    /// injects**, on the same screen as the bars it scales.
    var peakForceRegimeSentence: String {
        peakForceIsSharedBetweenLegs
            ? "Both legs' peak ground force is closed on the MEAN contact time, because this clip "
            + "cannot resolve the difference between them — so the left/right contact difference "
            + "is not inside these bars either."
            : String(format: "Each leg's peak ground force is closed on its OWN contact time, so "
                     + "%.0f%% of every bar's left/right difference is that contact-time "
                     + "difference re-expressed as force, on the %@. Every claim floor below is "
                     + "widened by that amount.",
                     abs(contactTimeContributionPercent),
                     contactTimeContributionPercent >= 0 ? "left" : "right")
    }

    /// What the muscle-to-muscle direction of this screen is, said where the
    /// numbers are. The list is ordered, and an ordered list of named muscles
    /// reads as a ranking unless it is told not to.
    ///
    /// The count is of the muscles ACTUALLY SHOWN, not of the model: a figure
    /// like "66 of 520" is about a file the user cannot see, and it would be
    /// wrong the moment the fallback model loads instead.
    var crossMuscleSentence: String {
        let shown = muscles.prefix(Self.displayedMuscleCount)
        let affected = shown.filter { !$0.pathIsModelled }.count
        let scope = affected == 0
            ? "the effort numbers are each on a scale of their own"
            : "\(affected) of the \(shown.count) muscles below are given a straight-line path "
            + "where the real tendon wraps around bone, so each one's effort number carries its "
            + "own unknown scale"
        return "These are ordered by which left/right comparison this clip resolved best — NOT by "
            + "which muscle worked hardest. That ordering is not available: \(scope). The scale "
            + "is the same on your left and right, so it cancels in the comparison below and only "
            + "there. The 3-D overlay picks its strongest muscles by the same uncalibrated "
            + "number, so read it as where load is, not as which muscle leads."
    }

    // MARK: - Construction

    /// Builds the summary from the frames a run produced. `nil` when the clip
    /// has no stance frame carrying muscle output.
    ///
    /// - Important: `frames` must be in capture order. Contacts are identified
    ///   by `GaitFrameOutcome.contactIndex`, which comes from the stance
    ///   intervals `GaitAnalysis` found — NOT from runs of consecutive frames.
    ///   Frames of one contact are grouped even if a solver-side hole separates
    ///   them, which is the whole point: a non-converged IK or a `submitAndWait`
    ///   timeout used to split one contact in two and have each half contribute
    ///   its own off-peak sample at double weight, invisibly to every counter.
    static func make(frames: [OfflineResultStore.FrameResult],
                     report: GaitReport,
                     framesPerSecond: Double,
                     filterTaps: Int) -> GaitLoadSummary? {
        /// One contact's usable samples, in capture order.
        struct Contact {
            var side: Int
            var samples: [[String: Double]] = []
        }
        var contacts: [Int: Contact] = [:]
        var contactOrder: [Int] = []
        var residuals: [Double] = []
        var disagreements = 0
        var doubleContacts = 0
        var noCleanWindow = 0
        var saturated = Set<String>()
        var saturatedBases = Set<String>()
        var flooredBases = Set<String>()
        var claimedStance = 0
        var usableStance = 0
        var usableLeft = 0
        var usableRight = 0

        for frame in frames {
            guard frame.isGaitStance,
                  let outcome = frame.motionState.gaitOutcome else { continue }

            claimedStance += 1
            if !outcome.contactDetectorsAgree { disagreements += 1 }
            if outcome.solverSawDoubleContact { doubleContacts += 1 }
            if !outcome.derivativeWindowInsideContact { noCleanWindow += 1 }

            // A frame whose two contact detectors disagree was solved with the
            // WRONG external load — `solveIDGRF` returns no ground force at all
            // when its geometric detector sees no foot down — and a frame whose
            // derivative window crossed a contact edge carries an acceleration
            // fitted across a discontinuity. Neither belongs in a load, and
            // neither belongs in the residual statistic.
            guard outcome.isUsableForLoadComparison else { continue }
            residuals.append(outcome.residualInBodyWeights)

            // A stance frame with no contact index is a contradiction — the plan
            // gave it a side but not the interval that side came from — so it is
            // not folded into a neighbouring contact, which is what keying on
            // adjacency used to do to every hole.
            guard let muscle = frame.muscleResult, outcome.contactIndex >= 0 else { continue }
            usableStance += 1
            let onLeft = outcome.contactSide < 0
            if onLeft { usableLeft += 1 } else { usableRight += 1 }

            let key = outcome.contactIndex
            if contacts[key] == nil {
                contacts[key] = Contact(side: outcome.contactSide)
                contactOrder.append(key)
            }
            contacts[key]?.samples.append(muscle.activations)

            // Both bounds are a warning about EVERY frame that went into the
            // contact, not only the sample that represents it: if the QP clipped
            // anywhere on this side, the linearity that makes a force error
            // cancel is gone for this muscle.
            for (name, activation) in muscle.activations {
                guard let (base, muscleSide) = split(name),
                      muscleSide == (onLeft ? "l" : "r") else { continue }
                if activation >= saturationThreshold {
                    saturated.insert(name)
                    saturatedBases.insert(base)
                } else if activation <= activationFloorThreshold {
                    flooredBases.insert(base)
                }
            }
        }

        guard claimedStance > 0 else { return nil }

        // Per side, per muscle: ONE value per contact — the middle of that
        // contact's usable samples (the mean of the two middle ones when there
        // is an even number, the convention `median(_:)` uses).
        var leftSamples: [String: [Double]] = [:]
        var rightSamples: [String: [Double]] = [:]
        var leftContactCount = 0
        var rightContactCount = 0
        for key in contactOrder {
            guard let contact = contacts[key], !contact.samples.isEmpty else { continue }
            let onLeft = contact.side < 0
            let m = contact.samples.count
            let middle = m % 2 == 1
                ? [contact.samples[m / 2]]
                : [contact.samples[m / 2 - 1], contact.samples[m / 2]]

            var sums: [String: Double] = [:]
            var counts: [String: Int] = [:]
            for sample in middle {
                for (name, activation) in sample {
                    sums[name, default: 0] += activation
                    counts[name, default: 0] += 1
                }
            }
            for (name, sum) in sums {
                guard let (base, muscleSide) = split(name) else { continue }
                // Only credit a muscle to the leg that is actually on the
                // ground. The swing leg is doing something, but it is not
                // carrying the contact, and mixing the two would make the
                // left/right comparison a comparison of phases.
                guard muscleSide == (onLeft ? "l" : "r") else { continue }
                let value = sum / Double(counts[name] ?? 1)
                if onLeft {
                    leftSamples[base, default: []].append(value)
                } else {
                    rightSamples[base, default: []].append(value)
                }
            }
            if onLeft { leftContactCount += 1 } else { rightContactCount += 1 }
        }

        let peaks = report.peakVerticalForceInBodyWeights
        let contactTimeContribution = contactTimePeakContributionPercent(peaks: peaks)

        let bilateral = Set(leftSamples.keys).intersection(rightSamples.keys)
        let loads = bilateral.compactMap { base -> MuscleLoad? in
            guard let l = leftSamples[base], let r = rightSamples[base],
                  !l.isEmpty, !r.isEmpty else { return nil }
            let leftLoad = mean(l), rightLoad = mean(r)
            guard leftLoad > 0 || rightLoad > 0 else { return nil }
            return MuscleLoad(
                id: base,
                displayName: prettyName(base),
                leftLoad: leftLoad,
                rightLoad: rightLoad,
                leftContacts: l.count,
                rightContacts: r.count,
                isSaturated: saturatedBases.contains(base),
                isAtActivationFloor: flooredBases.contains(base),
                samplingUncertaintyPercent: samplingUncertaintyPercent(left: l, right: r),
                pathIsModelled: !musclesWithUnmodelledPaths.contains(base))
        }

        let sortedResiduals = residuals.sorted()
        let maxResidual = sortedResiduals.last ?? 0
        let medianResidual = sortedResiduals.isEmpty
            ? 0 : sortedResiduals[sortedResiduals.count / 2]

        let taps = WindowedDerivativeFilter.admissibleTaps(filterTaps)
        let shortest = Double(report.shortestContactSamples) * report.sampleInterval

        var summary = GaitLoadSummary(
            muscles: loads,
            resolvableAsymmetryPercent: report.resolution.resolvableAsymmetryPercent,
            quantisationFloorPercent: report.resolution.quantisationFloorPercent,
            strideRepeatabilityPercent: report.resolution.strideRepeatabilityPercent,
            measuredStrideRepeatabilityPercent: report.resolution
                .measuredStrideRepeatabilityPercent,
            strideRepeatabilityBoundPercent: report.resolution.strideRepeatabilityBoundPercent,
            peakForceIsSharedBetweenLegs: report.peakVerticalForceIsSharedBetweenLegs,
            contactTimeContributionPercent: report.peakVerticalForceIsSharedBetweenLegs
                ? 0 : contactTimeContribution,
            framesPerContact: report.resolution.framesPerContact,
            framesPerSecond: framesPerSecond,
            stanceFrameCount: usableStance,
            claimedStanceFrameCount: claimedStance,
            saturatedMuscleCount: saturated.count,
            flooredMuscleCount: flooredBases.count,
            maxVerticalForceResidualInBodyWeights: maxResidual,
            medianVerticalForceResidualInBodyWeights: medianResidual,
            residualFrameCount: residuals.count,
            residualGatePassed: !residuals.isEmpty
                && maxResidual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
            contactDetectorDisagreements: disagreements,
            solverSawDoubleContactCount: doubleContacts,
            framesWithoutACleanDerivativeWindow: noCleanWindow,
            leftStanceFrameCount: usableLeft,
            rightStanceFrameCount: usableRight,
            leftContactCount: leftContactCount,
            rightContactCount: rightContactCount,
            horizontalRootAccelerationModelled: false,
            derivativeFilterTaps: taps,
            derivativeFilterSpanMilliseconds: 1000 * Double(taps - 1) * report.sampleInterval,
            shortestContactMilliseconds: 1000 * shortest,
            derivativeNoiseAmplification: WindowedDerivativeFilter
                .accelerationNoiseAmplification(taps: taps))
        summary.muscles = summary.ordered(loads)
        return summary
    }

    /// **The ordering, which is a statement about the CLAIMS and not about the
    /// muscles.** Publishable comparisons first, each by how far it clears its
    /// own floor; then the rest, by the same measure. Ties break on the id so
    /// the list is deterministic.
    ///
    /// Every key here is a ratio of two of the muscle's own numbers, so — like
    /// `differencePercent` itself — it is invariant to that muscle's moment-arm
    /// scale. An ordering by `max(leftLoad, rightLoad)` is not, which is why it
    /// is gone.
    private func ordered(_ loads: [MuscleLoad]) -> [MuscleLoad] {
        func headroom(_ load: MuscleLoad) -> Double {
            let floor = claimFloorPercent(for: load)
            guard floor.isFinite, floor > 0, load.differencePercent.isFinite else { return -.infinity }
            return abs(load.differencePercent) / floor
        }
        return loads.sorted { a, b in
            let pa = permits(a), pb = permits(b)
            if pa != pb { return pa }
            let ha = headroom(a), hb = headroom(b)
            if ha != hb { return ha > hb }
            return a.id < b.id
        }
    }

    /// `100·(Fmax_L − Fmax_R)/mean` — exactly the left/right difference that
    /// per-leg force scaling puts into every muscle when the activations are
    /// identical.
    static func contactTimePeakContributionPercent(peaks: Bilateral<Double>) -> Double {
        let l = peaks.left, r = peaks.right
        let m = 0.5 * (l + r)
        guard l.isFinite, r.isFinite, m > 0 else { return 0 }
        return 100 * (l - r) / m
    }

    /// The 95 % half-width of `differencePercent` from the per-contact samples
    /// alone. Infinite when either side has fewer than two contacts.
    static func samplingUncertaintyPercent(left: [Double], right: [Double]) -> Double {
        let nL = left.count, nR = right.count
        guard nL >= 2, nR >= 2 else { return .infinity }
        let mL = mean(left), mR = mean(right)
        let m = 0.5 * (mL + mR)
        guard m > 0 else { return .infinity }
        let standardError = (variance(left) / Double(nL) + variance(right) / Double(nR))
            .squareRoot()
        return tMultiplier(degreesOfFreedom: Swift.min(nL, nR) - 1) * 100 * standardError / m
    }

    /// Two-sided 95 % Student-t multiplier. Conservative above 15 degrees of
    /// freedom (it holds 2.131 rather than falling to 1.96), because a clip has
    /// at most a handful of contacts per side and the interesting regime is the
    /// small one — at `df = 1` the factor is 12.7, not 2, and a claim built on
    /// two contacts a side has to say so.
    static func tMultiplier(degreesOfFreedom df: Int) -> Double {
        let table = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306,
                     2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131]
        guard df >= 1 else { return .infinity }
        return df <= table.count ? table[df - 1] : table[table.count - 1]
    }

    static func mean(_ values: [Double]) -> Double {
        values.isEmpty ? 0 : values.reduce(0, +) / Double(values.count)
    }

    /// Sample variance, `n − 1` denominator. Zero for fewer than two samples,
    /// which callers must not read as "no scatter" — they check the count.
    static func variance(_ values: [Double]) -> Double {
        guard values.count > 1 else { return 0 }
        let m = mean(values)
        return values.map { ($0 - m) * ($0 - m) }.reduce(0, +) / Double(values.count - 1)
    }

    /// The activation at which the QP's `a ≤ 1` bound is treated as reached,
    /// **derived from the solver's own tolerances rather than picked**.
    ///
    /// This was 0.999, which is finer than OSQP's own convergence band: with
    /// `eps_abs = eps_rel = 1e-3`, `A = I`, `z ∈ [0.02, 1]` and
    /// `OSQP_SOLVED_INACCURATE` accepted (ten times looser), a genuinely clipped
    /// activation returns as low as 0.98. So `isSaturated` read false for
    /// muscles sitting on the bound, `%.2f` printed them as "1.00", and the one
    /// gate protecting the ratio argument was passing exactly the cases it
    /// exists to catch.
    /// A `let`, not a computed property: it is read once per muscle per frame
    /// inside `make`, and the floor below used to allocate a `MuscleSolver` to
    /// answer.
    static let saturationThreshold: Double =
        MuscleSolver.maxActivation - MuscleSolver.saturationActivationTolerance

    /// The activation at or below which the QP's `a ≥ aMin` bound is treated as
    /// reached, by the same tolerance argument as `saturationThreshold`. An
    /// activation of 0.03 against a 0.02 bound is not distinguishable from the
    /// bound by a solver whose own band is 0.02 wide.
    static let activationFloorThreshold: Double =
        MuscleSolver().minActivation + MuscleSolver.saturationActivationTolerance

    /// `"glmax1_r"` → `("glmax1", "r")`. Nil for a muscle with no side, which
    /// is every trunk and spine muscle in `FullBody.osim`.
    static func split(_ name: String) -> (base: String, side: String)? {
        guard name.count > 2 else { return nil }
        let suffix = String(name.suffix(2))
        guard suffix == "_l" || suffix == "_r" else { return nil }
        return (String(name.dropLast(2)), String(suffix.dropFirst()))
    }

    /// **Base names whose `<PathWrapSet>` this build does not model**, so their
    /// moment arm — and every activation derived from it — carries a scale error
    /// of its own.
    ///
    /// Written in the names that reach this type, i.e. after
    /// `NimbleEngine.displayMuscleAliases` has merged `vaslat140 → vaslat` and
    /// `gaslat140 → gaslat`. It is the union over both bundled models
    /// (`FullBody.osim`, 66 muscles / 76 wrap references, and the
    /// `Rajagopal2016.osim` fallback, 42 / 46), and
    /// `MomentArmTests.testTheUnmodelledWrapTableMatchesTheShippedModels`
    /// checks it against what the parser actually reports at runtime — so
    /// swapping a model cannot leave this list quietly wrong.
    static let musclesWithUnmodelledPaths: Set<String> = [
        // Shoulder and arm (FullBody only)
        "ANC", "BIClong", "BICshort", "BRD", "SUP",
        "TR2", "TR3", "TR4", "TR5", "TRIlat", "TRIlong", "TRImed",
        // Hip and thigh
        "addbrev", "addlong", "addmagDist", "addmagIsch", "addmagMid", "addmagProx",
        "glmax1", "glmax2", "glmax3", "grac", "iliacus", "psoas",
        "recfem", "semimem", "semiten",
        "vasint", "vasmed", "vaslat",
        // Shank
        "bfsh", "bfsh140", "gasmed", "gaslat",
    ]

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
