import Foundation

/// The RELATIVE view of a running clip — **left against right, one muscle at a
/// time** — plus the per-clip resolution that says which of those comparisons
/// the clip is actually allowed to make.
///
/// # ⚠️ The per-muscle left/right claim is RETIRED in this build
///
/// This type still MEASURES it, because the measurement is what shows it cannot
/// be stated. `perMuscleLeftRightClaimIsSupported` is `false`, `permits(_:)`
/// therefore returns false for every muscle on every clip, and no screen prints
/// a per-muscle percentage. `clearsStatisticalFloor(_:)` is the surviving
/// statistical predicate and is deliberately NOT permission to say anything.
///
/// # Why — the cancellation argument was circular, and its replacement fails
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
/// Until 2026-08-08 this file said the error was a per-muscle SCALE that
/// cancelled out of the left/right ratio, and cited a measurement. **That
/// measurement could not have come out any other way.** The rig it ran on made
/// every RIGHT joint torque `0.8×` its left counterpart, and the QP
/// (`min ½aᵀ(εI + λAᵀA)a − λτᵀAa`) is LINEAR in `τ` while no bound is active —
/// so `a_R = 0.8·a_L` exactly, for ANY moment-arm matrix. The perturbation could
/// not move the answer; in exact arithmetic it moves it by 1e-6 pp, and the
/// 1.04 pp that was reported is the solver's own tolerance. See
/// `MomentArmErrorCancellationTests`, which now states that as an identity and
/// tests the case that can actually fail.
///
/// Two measurements replace it, both on the shipping OSQP solver:
///
/// 1. **Where the two legs' torques ARE proportional, every muscle reads the
///    SAME left/right figure.** That is the same linearity. So in the only
///    regime where the moment-arm error provably cancels, the per-muscle
///    breakdown carries no per-muscle information at all — it is one number
///    repeated, and that number is the torque scale ratio, which the contact
///    block above the list already reports from timing.
/// 2. **Where they differ in SHAPE — which is what a gait asymmetry IS — the
///    error does not cancel.** Perturbing one muscle's moment arm by ×0.6 on
///    BOTH sides, with the right leg at 0.8× the left's hip torque and 1.0× its
///    knee torque, moves a published left/right figure by **9.92 pp** on the
///    shipping OSQP solver — a real `−17.89 %` displayed as `−7.97 %` — against
///    a solver noise floor of 1.52 pp. That is larger than the finest of the
///    three pinned clips' publication floors (8.086 %) and 98 % of the next
///    (10.145 %); in exact arithmetic, where the solver's own tolerance does not
///    blunt it, the same case is 13.11 pp, and a larger shape difference
///    (knee_r/knee_l = 0.6) measures 17.72 pp.
///
///    It moves `beta`, a muscle whose OWN path is modelled correctly, because
///    the QP redistributes load between synergists — so no per-row "this
///    muscle's path is a straight line" flag can contain it.
///
/// Together: all per-muscle differentiation in this statistic lives in the
/// non-proportional part of the torque, and that is exactly the part an
/// unmodelled wrap corrupts. The claim is not gateable, because the gate that
/// would protect it (proportional torques) is the regime in which it says
/// nothing.
///
/// # Why there is no headline newton figure here, and never will be
///
/// The peak ground force this pipeline computes comes from contact and flight
/// TIMING (`GaitForceModel`), and its absolute value carries a criterion-
/// dependent bias of roughly ±28 ms of contact time — about 18 % on `Fmax` at
/// the owner's cadences. Printed on a screen, "2.9 body weights" reads as a
/// measurement. It is not one.
///
/// A peak-force error of that kind is a COMMON SCALE on `τ`, and the QP is
/// linear in `τ` while no bound is active, so it divides out of a left/right
/// ratio exactly — unlike a moment-arm error, which changes the MATRIX and
/// therefore does not. That distinction is the whole content of the section
/// above: `τ → sτ` cancels, `A → DA` does not.
///
/// # The statistical floor passes three terms, not one
///
/// `claimFloorPercent(for:)` is what a left/right difference would have to
/// clear for the STATISTICS to allow it — which, since the paragraph above,
/// is a necessary and not a sufficient condition. It is built from three
/// separate things:
///
/// 1. **What the clip's sampling grid and this runner's strides allow** —
///    `GaitResolution.resolvableAsymmetryPercent`. Timing only.
/// 2. **What this muscle's own step-to-step scatter allows** —
///    `MuscleLoad.samplingUncertaintyPercent`, the half-width of
///    `differencePercent` computed from the per-contact samples themselves.
///    This term did not exist, and its absence was measurable: at the scatter
///    `GaitLoadStatisticTests` uses, a perfectly symmetric runner produced a
///    per-clip standard deviation of 9.47 % against a 10.145 % publication
///    floor, i.e. roughly one in four muscles reading a false finding. A floor
///    derived from frames per contact and the stride period cannot see that,
///    because activation scatter is not one of its inputs.
///    **Its confidence level is family-wise, not per-comparison** — see
///    `screenedComparisonCount`. `make` builds one `MuscleLoad` for every
///    bilateral pair the model carries (175 of `FullBody.osim`'s 520 muscles
///    have both an `_l` and an `_r`), and the list is then sorted BY the
///    statistic under test, so quoting a per-comparison 95 % against the top of
///    that list understates the false-positive rate by the size of the pool.
/// 3. **The contact-time difference the force model injects** —
///    `contactTimeContributionPercent`. Where the clip resolves the left/right
///    contact difference, `Fmax_side = (π/2)(1 + tf/tc_side)` differs between
///    the legs, the QP is linear in the external load, and that difference
///    lands inside every muscle's left/right number as a common additive term.
///
/// # The clip-level gates
///
/// `arePublishable` is the single answer to "did this clip measure loads at
/// all". It is a DATA gate, and it is no longer the last one: since the
/// per-muscle comparison was retired, a clip can pass every gate below and still
/// state no muscle finding. The 3-D overlay is off on the running path for the
/// same reason (`OfflinePlaybackView.muscleMagnitudesArePublishable`).
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
        /// **The FAMILY-WISE 95 % half-width of `differencePercent` implied by
        /// this muscle's own contact-to-contact scatter**, in percent.
        ///
        /// `SE = √(s_L²/n_L + s_R²/n_R)` over the per-contact samples, scaled to
        /// the same denominator `differencePercent` uses and multiplied by the
        /// two-sided Student-t factor for `min(n_L, n_R) − 1` degrees of freedom
        /// — Student-t, not 1.96, because at five or six contacts a normal
        /// multiplier understates the interval by a third.
        ///
        /// **The t is taken at `α / N`, not at `α`.** `N` is
        /// `screenedComparisonCount`, the number of bilateral pairs this clip
        /// screened — every pair the model carries, ~175 on `FullBody.osim`.
        /// A per-comparison 95 % interval applied to ~175 comparisons and then
        /// sorted by the statistic puts the largest order statistic first by
        /// construction; the interval has to be the one that holds for the whole
        /// family, or the number quoted beside it is not the error rate the user
        /// is exposed to. Measured cost: see `GaitClaimSurvivalTests`.
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

    }

    /// Bilateral muscles, **ordered by how well this clip resolved each one's
    /// left/right difference** — the ones clearing their statistical floor
    /// first, then by the size of the difference relative to that floor.
    ///
    /// ⚠️ **This ordering is BY the statistic, so the head of it is an order
    /// statistic and not a sample.** Nothing may take the top `k` of this array
    /// and describe them with a per-comparison error rate; that is what the
    /// panel did, over a pool of ~175 pairs, while telling the user "about one
    /// in twenty". No screen consumes the order any more —
    /// `perMuscleLeftRightClaimIsSupported` is false — and the correction that
    /// makes the interval hold over the whole pool lives in
    /// `samplingUncertaintyPercent`, so a future consumer inherits it rather
    /// than having to remember.
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
    /// **What the CONTACT-TIME claim has to clear** — copied from
    /// `GaitReport.contactClaimFloorPercent`, which is
    /// `max(resolvableAsymmetryPercent, contactSamplingUncertaintyPercent)`.
    ///
    /// It is here so `resolutionSentence` can stop telling the user that the
    /// timing figure is the one the comparison below has to clear. It was not,
    /// and on a 30 fps clip it is the smaller of the two.
    let contactClaimFloorPercent: Double
    /// The contact durations' own sampling half-width — the term the timing
    /// floor never contained. See `GaitReport.contactSamplingUncertaintyPercent`.
    let contactSamplingUncertaintyPercent: Double
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
    /// **How many bilateral comparisons this clip SCREENED** — the family the
    /// per-muscle confidence intervals have to hold over.
    ///
    /// It is every muscle that could have produced a claim: both sides carry at
    /// least `minimumContactsPerSide` contacts, neither bound is active, and the
    /// mean is positive. It deliberately does NOT depend on the size of any
    /// muscle's difference, because a family defined by the statistic under test
    /// is not a family — `ordered(_:)` sorts by exactly that statistic and the
    /// panel used to take the top of it.
    ///
    /// On `FullBody.osim` the ceiling is 175 (the pairs with both an `_l` and an
    /// `_r` out of 520 muscles); the realised number is lower on any clip where
    /// the QP pins muscles to a bound, which it does to most of them.
    let screenedComparisonCount: Int
    /// How many MUSCLES hit the `a ≤ 1` bound anywhere.
    ///
    /// Muscles, not muscle-sides: it counted `soleus_l` and `soleus_r` as two
    /// while `flooredMuscleCount` beside it counted soleus once, and the panel
    /// printed the pair in one sentence — so a clip that clipped ten muscles on
    /// both legs told the user twenty of their muscles had maxed out.
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
    /// The family-wise error rate the per-muscle intervals are built to hold at.
    /// `samplingUncertaintyPercent` takes its Student-t at `α / N` for `N =`
    /// `screenedComparisonCount`, so this is the probability that ANY of the
    /// clip's comparisons reads a difference that is not there — not the
    /// probability for one of them.
    static let familyWiseErrorRate = 0.05

    /// **Whether a per-muscle left/right difference may be STATED.** `false`,
    /// and not because of anything a clip can change.
    ///
    /// The type doc carries the two measurements. In short: the QP is linear in
    /// the joint torques, so where the two legs' torques are proportional every
    /// muscle reads the same figure (no per-muscle information) and where they
    /// are not, an unmodelled `PathWrap` moves a published figure by 9.92 pp on
    /// the shipping solver — past the finest floor the pinned clips carry —
    /// including on muscles whose own path is modelled correctly, because the QP
    /// redistributes load between synergists.
    ///
    /// **What would flip it back.** Model the 76 missing `PathWrap` references
    /// (or bound their moment-arm error), and then re-run
    /// `MomentArmErrorCancellationTests.testAShapeAsymmetryMakesABilateralMomentArmErrorLeak`
    /// with the bound in place: if the leak drops below the smallest publication
    /// floor on the pinned clips (8.086 %), the claim can come back with that
    /// term added to `claimFloorPercent`. Nothing else does it — more contacts
    /// do not, because the leak is a bias and not scatter.
    static let perMuscleLeftRightClaimIsSupported = false

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

    /// **What re-filming does NOT buy**, stated on the same screen as the
    /// refusal that offers it. Nil once a per-muscle claim is supported again,
    /// at which point every lever in `withheldReason` can deliver rows and the
    /// sentence would be false.
    ///
    /// It exists because `withheldReason`'s levers are all real levers for the
    /// DATA and none of them is a lever for the ROWS: `arePublishable` gates
    /// nothing else the user can see, so passing the gate changes one paragraph.
    /// Offering "film a steadier, straighter run" without this is selling a
    /// re-shoot against a limit no shoot can move.
    var muscleRowsUnaffectedByRefilmingSentence: String? {
        Self.perMuscleLeftRightClaimIsSupported
            ? nil
            : "Filming again would fix the measurement in that line. It would not produce the "
            + "muscle comparison — that limit is the model's, not this clip's, and it is the "
            + "paragraph above."
    }

    /// Why the loads are withheld, naming the measurement AND the lever. Nil
    /// when they are not.
    ///
    /// ⚠️ **The lever here is a lever for the DATA.** It is not a lever for the
    /// per-muscle rows, and the panel must not present it as one — see
    /// `muscleRowsUnaffectedByRefilmingSentence` and `GaitReportPanel.loadBlock`.
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

    /// **The STATISTICAL floor this muscle's left/right difference has to
    /// clear** — necessary, and since the moment-arm leak, not sufficient.
    ///
    /// `max(what the clip's timing resolves, what this muscle's own step-to-step
    /// scatter resolves across the whole screened family) + |the contact-time
    /// term the force model injects|`. The first two are competing lower bounds
    /// on a distinguishable difference; the third is a known additive
    /// contamination that has to be taken off before what is left can be
    /// attributed to the muscle.
    ///
    /// The unmodelled-`PathWrap` leak is deliberately NOT a fourth term here.
    /// It is not a floor: nothing in this pipeline measures or bounds it, so
    /// adding an invented number would make the gate look quantitative when it
    /// is not. It is handled where an unbounded error belongs — as a flat
    /// refusal, `perMuscleLeftRightClaimIsSupported`.
    func claimFloorPercent(for load: MuscleLoad) -> Double {
        Swift.max(resolvableAsymmetryPercent, load.samplingUncertaintyPercent)
            + abs(contactTimeContributionPercent)
    }

    /// **Whether this muscle's left/right difference clears everything the
    /// STATISTICS ask of it — which is not permission to say it.** See
    /// `permits(_:)`.
    ///
    /// It takes the MUSCLE, not a bare percentage: the floor depends on that
    /// muscle's own contact-to-contact scatter, and a signature that accepts a
    /// lone number cannot ask about it. The old one could not, which is exactly
    /// how a claim came to be gated on timing quantisation alone.
    func clearsStatisticalFloor(_ load: MuscleLoad) -> Bool {
        let d = load.differencePercent
        return d.isFinite
            && !load.isSaturated
            && !load.isAtActivationFloor
            && abs(d) >= claimFloorPercent(for: load)
            && arePublishable
    }

    /// **Whether this muscle's left/right difference may be put in front of the
    /// user.** `false` for every muscle on every clip in this build — the
    /// statistical floor above is necessary, and
    /// `perMuscleLeftRightClaimIsSupported` is the sufficient condition it does
    /// not have.
    func permits(_ load: MuscleLoad) -> Bool {
        Self.perMuscleLeftRightClaimIsSupported && clearsStatisticalFloor(load)
    }

    /// **Why no muscle row appears on screen**, in the user's words. One
    /// paragraph, on the screen where the rows used to be.
    ///
    /// It names what WAS measured, so the absence does not read as the app
    /// having failed to run, and it points at the contact-time comparison above
    /// it, which is the left/right finding this clip can still support.
    var perMuscleRetirementSentence: String {
        "Left and right were compared for \(muscles.count) muscle pairs, and none of those "
        + "comparisons is shown. The muscle model reaches each muscle's effort by dividing a "
        + "joint moment by a moment arm, and 66 of its muscles are given a straight line where "
        + "the real tendon wraps around bone. That error cancels out of a left/right "
        + "comparison only when your two legs load the joints in the same PATTERN — and when "
        + "they do, every muscle returns the same figure, so there is one number for the whole "
        + "leg and no per-muscle finding in it. When they do not — which is what a left/right "
        + "difference IS — the error moves a muscle's figure by around 10 percentage points on "
        + "this app's own test rig, as large as the difference a good clip can resolve, and it "
        + "moves muscles whose own paths are modelled correctly. So the contact-time comparison above is the "
        + "left/right finding this clip supports, and a per-muscle breakdown is not."
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
    ///
    /// # It no longer says the timing figure is what the comparison must clear
    ///
    /// It did, and that was the user-facing half of the blocker: the sentence
    /// pointed at ±8 % while `asymmetryClaim` published against ±8 % and the
    /// statistic's own scatter was ±20 %. The timing number is still first,
    /// because it is the only one with a camera attached; the floor that
    /// actually governs is named beside it whenever the two differ.
    var resolutionSentence: String {
        let base = String(format: "This clip's timing resolves left/right to about ±%.0f%% "
                          + "(%.1f frames per contact at %.0f fps).",
                          resolvableAsymmetryPercent, framesPerContact, framesPerSecond)
        // The camera advice below is about the QUANTISATION FLOOR. When the
        // contact durations' own scatter is what binds, a faster camera cannot
        // deliver the claim, so the promise is withheld and the reason named —
        // the same rule the refusals follow: only offer a lever when one exists.
        guard contactClaimFloorPercent <= resolvableAsymmetryPercent + 0.5 else {
            return base + String(format: " But the contact-time comparison below has to clear "
                                 + "±%.0f%%: your own contact times varied from step to step by "
                                 + "more than the sampling grid explains, that variation is inside "
                                 + "the difference being measured, and no frame rate removes it.",
                                 contactClaimFloorPercent)
        }
        let withFloor = base + " That is what the CONTACT-TIME comparison below has to clear."
        let target = Swift.max(5.0, bestAchievablePercentAtAnyFrameRate)
        guard target.isFinite, resolvableAsymmetryPercent > target else { return withFloor }
        let needed = frameRateNeeded(forPercent: target)
        guard needed.isFinite, needed > framesPerSecond,
              needed <= FrameSource.highestAnalysableFrameRate else {
            return withFloor + " A higher frame rate is the only lever, and this clip is already "
                 + "at what the analysis window can cover."
        }
        return withFloor + String(format: " Filming at %.0f fps would resolve ±%.0f%%.",
                                  needed, target)
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
    /// **It was also claimed to cancel out of a left/right comparison of one
    /// muscle**, "for the same reason a moment-arm error does". That reason is
    /// gone: a moment-arm error does not cancel out of a left/right comparison
    /// unless the two legs' torques are proportional, and where they are, the
    /// comparison carries no per-muscle information. The sentence no longer
    /// makes the claim.
    var unmodelledTermSentence: String {
        "Fore-aft braking and push-off is not measured — and it is not left out either. Inverse "
        + "dynamics assigns whatever horizontal ground force makes the body match a pelvis the "
        + "pose model holds still, so every joint moment here contains a fabricated fore-aft "
        + "term of roughly 0.2-0.35 body weights. It changes through the contact, so it is not "
        + "a single scale that divides out of a comparison between two muscles, or between your "
        + "two legs when they load the joints differently."
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
    /// injects, in the unit it is in**.
    ///
    /// It used to say "%.0f%% of every bar's left/right difference is that
    /// contact-time difference re-expressed as force" — the grammar of a SHARE
    /// for a number that is an additive offset in percentage POINTS. On the
    /// panel's own worked example (tcL 200 ms, tcR 160 ms) a muscle reading
    /// 13 % harder on the right came with "9 % of the difference is contact
    /// time", from which a reader computes 1.2 points of artefact where the
    /// truth is 9.4 — a 3.3× overestimate of their own asymmetry, in the
    /// direction that flatters the finding.
    var peakForceRegimeSentence: String {
        peakForceIsSharedBetweenLegs
            ? "Both legs' peak ground force is closed on the MEAN contact time, because this clip "
            + "cannot resolve the difference between them — so the left/right contact difference "
            + "is not inside the muscle numbers either."
            : String(format: "Each leg's peak ground force is closed on its OWN contact time, "
                     + "which puts %.0f percentage points of %@-high difference into every "
                     + "muscle's left/right figure before any muscle has done anything — "
                     + "subtract it, do not take a share of it.",
                     abs(contactTimeContributionPercent),
                     contactTimeContributionPercent >= 0 ? "left" : "right")
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

        // TWO passes, because a confidence interval needs to know how many
        // comparisons it is one of, and that count must not depend on the
        // statistic being tested. Pass 1 counts the pairs this clip SCREENED:
        // both sides carried enough contacts, neither bound was active, the mean
        // is positive. Nothing here looks at how big any difference is.
        var screened = 0
        for base in bilateral {
            guard let l = leftSamples[base], let r = rightSamples[base],
                  l.count >= minimumContactsPerSide, r.count >= minimumContactsPerSide,
                  !saturatedBases.contains(base), !flooredBases.contains(base) else { continue }
            if mean(l) + mean(r) > 0 { screened += 1 }
        }

        // Pass 2 builds the loads, each carrying the interval that holds across
        // all `screened` of them at once.
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
                samplingUncertaintyPercent: samplingUncertaintyPercent(left: l, right: r,
                                                                       comparisons: screened),
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
            contactClaimFloorPercent: report.contactClaimFloorPercent,
            contactSamplingUncertaintyPercent: report.contactSamplingUncertaintyPercent,
            peakForceIsSharedBetweenLegs: report.peakVerticalForceIsSharedBetweenLegs,
            contactTimeContributionPercent: report.peakVerticalForceIsSharedBetweenLegs
                ? 0 : contactTimeContribution,
            framesPerContact: report.resolution.framesPerContact,
            framesPerSecond: framesPerSecond,
            stanceFrameCount: usableStance,
            claimedStanceFrameCount: claimedStance,
            screenedComparisonCount: screened,
            saturatedMuscleCount: saturatedBases.count,
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
            let pa = clearsStatisticalFloor(a), pb = clearsStatisticalFloor(b)
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

    /// The half-width of `differencePercent` from the per-contact samples alone,
    /// at a confidence level that holds across all `comparisons` of this clip's
    /// screened pairs at once. Infinite when either side has fewer than two
    /// contacts.
    ///
    /// - Parameter comparisons: `screenedComparisonCount`. One means "this is
    ///   the only comparison being made", which is the assumption the shipped
    ///   code used to make silently while screening ~175 of them.
    /// The estimator itself lives in `MeanDifferenceUncertainty`, because the
    /// CONTACT-TIME claim needs the identical quantity and was shipped for two
    /// days without it — gated on this clip's timing while its own statistic is
    /// a difference of two means of contact durations. One implementation, two
    /// consumers.
    static func samplingUncertaintyPercent(left: [Double], right: [Double],
                                           comparisons: Int = 1) -> Double {
        MeanDifferenceUncertainty.halfWidthPercent(left: left, right: right,
                                                   comparisons: comparisons,
                                                   alpha: familyWiseErrorRate)
    }

    /// **Two-sided Student-t multiplier, Bonferroni-corrected over
    /// `comparisons`.**
    ///
    /// `comparisons = 1` reproduces the 95 % table this used to be, to three
    /// decimals, and `MuscleUncertaintyTests` pins that against the published
    /// values so the change of method cannot move the old numbers.
    ///
    /// Bonferroni rather than Šidák: the comparisons share a clip, a pose
    /// stream and a force model, so they are positively dependent, and Šidák's
    /// independence assumption is not one this pipeline can support. The
    /// difference at `N = 175` is under 1 % of the multiplier anyway.
    ///
    /// **Degrees of freedom are clamped at 15**, which is what the old table
    /// did by ending there. It only ever binds above 16 contacts on a side —
    /// four times what a 4 s clip carries — and clamping keeps this function
    /// from returning anything NARROWER than the table it replaces.
    static func tMultiplier(degreesOfFreedom df: Int, comparisons: Int = 1) -> Double {
        MeanDifferenceUncertainty.tMultiplier(degreesOfFreedom: df, comparisons: comparisons,
                                              alpha: familyWiseErrorRate)
    }

    static func mean(_ values: [Double]) -> Double {
        MeanDifferenceUncertainty.mean(values)
    }

    /// Sample variance, `n − 1` denominator. Zero for fewer than two samples,
    /// which callers must not read as "no scatter" — they check the count.
    static func variance(_ values: [Double]) -> Double {
        MeanDifferenceUncertainty.variance(values)
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
    /// **Written in BOTH namespaces, because the shipping path uses the raw one
    /// and this table used to be in the other.** `NimbleEngine` puts the RAW
    /// solver name into `SolveRecord` (`displayMuscle` is a separate value, used
    /// only by the live bar), so `make` splits `vaslat140_r` and looks up
    /// `vaslat140` — while this table listed only `vaslat`, the form
    /// `displayMuscleAliases` produces. Vastus lateralis and lateral
    /// gastrocnemius, two of the muscles a runner would look for first, were
    /// therefore recorded as having modelled paths on the production model. The
    /// guard test passed because it applied the alias transform before
    /// comparing — a transform the shipping path never performs.
    ///
    /// It is the union over both bundled models (`FullBody.osim`, 66 muscles /
    /// 76 wrap references, and the `Rajagopal2016.osim` fallback, 42 / 46) in
    /// both namespaces, and
    /// `MomentArmTests.testTheUnmodelledWrapTableMatchesTheShippedModels`
    /// checks it against what the parser actually reports at runtime, WITHOUT
    /// the alias transform — so swapping a model cannot leave this list quietly
    /// wrong.
    static let musclesWithUnmodelledPaths: Set<String> = [
        // Shoulder and arm (FullBody only)
        "ANC", "BIClong", "BICshort", "BRD", "SUP",
        "TR2", "TR3", "TR4", "TR5", "TRIlat", "TRIlong", "TRImed",
        // Hip and thigh
        "addbrev", "addlong", "addmagDist", "addmagIsch", "addmagMid", "addmagProx",
        "glmax1", "glmax2", "glmax3", "grac", "iliacus", "psoas",
        "recfem", "semimem", "semiten",
        "vasint", "vasmed", "vaslat", "vaslat140",
        // Shank
        "bfsh", "bfsh140", "gasmed", "gaslat", "gaslat140",
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
