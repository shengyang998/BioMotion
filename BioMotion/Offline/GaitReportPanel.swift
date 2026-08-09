import SwiftUI

/// What a running clip is allowed to tell the user, and what it has to refuse.
///
/// Layout order is the deliberate part. The resolution line comes FIRST — before
/// any comparison — because it is the number that decides whether every other
/// line on this screen is a finding or noise, and because it is the only line
/// with an action attached ("film at a higher frame rate").
///
/// There is no newton figure anywhere on this screen. See `GaitLoadSummary`.
///
/// # The loads are drawn INSIDE the gate, not beside it
///
/// `loadBlock` used to render every muscle's L/R peak and bars unconditionally
/// while `honestyBlock`, three lines below, printed "FAILED, loads withheld".
/// `summary.arePublishable` decides the whole block, and the withheld case
/// states the measurement and the lever. The 3-D overlay in
/// `OfflinePlaybackView` is stricter than that gate rather than equal to it: it
/// draws no muscles at all on an analysed running clip, because colouring the
/// strongest 24 is a cross-muscle ordering and there is no muscle claim left on
/// this path to draw.
///
/// ⚠️ **Since 2026-08-08 that gate no longer chooses BETWEEN two answers.** The
/// permanent reason (`perMuscleRetirementSentence`) prints on every branch and
/// the data refusal prints after it when it fires, because an `if/else` here
/// answered the header's own question — "why" — with a fixable property of the
/// clip on exactly the clips where the answer is "the model cannot do it". See
/// `loadBlock`.
///
/// # This screen makes ONE kind of comparison, and it is not a muscle one
///
/// **Left against right, by CONTACT TIME.** The per-muscle left/right rows are
/// gone as of 2026-08-08, with the cross-muscle ranking that went before them.
/// `MomentArmErrorCancellationTests` measures the first half of why: an error in
/// the matrix cancels out of a left/right comparison only when the two legs load
/// the joints in proportion — and in that regime every muscle returns the same
/// figure, so there is no per-muscle finding to make.
///
/// The rest of that paragraph used to say the error was the 66 straight-line
/// muscle paths, worth 9.92 pp. **Both numbers are historical.** Every `PathWrap`
/// is solved since 2026-08-08, and the 2026-08-09 re-measurement on real geometry
/// puts the moment-arm leak at a median of 0.977 pp and a worst case of
/// 123.10 pp, while the number this panel would print is out by a median of
/// 1.045 pp and a worst case of 108.58 pp against an 8.086 % floor. (The QP's own
/// termination slack was the dominant term for one commit, 14.88 pp at fixed
/// geometry; it is 4.4994e-05 pp since `scaling = 0` and `polishing = 1`.) It
/// still lands on muscles whose own paths are modelled correctly, because the QP
/// redistributes between synergists. See
/// `GaitLoadSummary.perMuscleLeftRightClaimIsSupported` for the gates.
///
/// So `loadBlock` states what was measured and why it is not shown, in one
/// paragraph, and points at the contact-time comparison above it — which is
/// measured from stance timing and touches neither a moment arm nor the QP.
///
/// # The honesty note is not optional on this path either
///
/// `PostureFindings.alwaysVisibleNote` ("No normal range is applied. These are
/// measurements, not diagnoses…") is rendered unconditionally by
/// `PostureFindingsPanel`. This panel REPLACES that panel on an analysed
/// running clip while making a strictly more clinical-sounding claim — named
/// anatomy, a 0-1 effort figure per side, a left/right verdict in warning
/// orange — so it carries the same note, in the same always-visible position.

struct GaitReportPanel: View {
    let outcome: OfflineResultStore.GaitOutcome
    let summary: GaitLoadSummary?

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 12) {
                switch outcome {
                case .notAttempted(let reason):
                    header("Not analysed as running")
                    Text(reason)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Text(NimbleEngine.MotionVerdict.gaitRefused.advice)
                        .font(.caption)

                case .refused(let report):
                    header("Not measured as running")
                    // Each refusal names its OWN cause and its OWN lever. One
                    // hard-coded sentence used to be printed under all of them,
                    // so a clip refused for 2.8 frames per contact — where the
                    // only lever is a faster camera, a number the app already
                    // has — was told to film a longer run at a steady pace, and
                    // re-filming that way produced byte-identical output.
                    ForEach(Array(report.refusals.enumerated()), id: \.offset) { _, refusal in
                        VStack(alignment: .leading, spacing: 2) {
                            Label(refusal.description, systemImage: "exclamationmark.triangle")
                                .font(.caption)
                                .foregroundStyle(.orange)
                            Text(refusal.advice(framesPerSecond: report.framesPerSecond))
                                .font(.caption2)
                                .foregroundStyle(.secondary)
                        }
                    }
                    flags(report)

                case .analysed(let report, _, _):
                    if let summary {
                        resolutionBlock(summary)
                        Divider()
                        contactBlock(report, summary)
                        Divider()
                        loadBlock(summary)
                        Divider()
                        honestyBlock(summary)
                    } else {
                        header("Running, no stance frame solved")
                        Text("The strides were measured but no contact produced muscle output.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    flags(report)
                }
                notDiagnosisNote
            }
            .padding(.horizontal)
            .padding(.vertical, 10)
        }
    }

    // MARK: Blocks

    private func header(_ text: String) -> some View {
        Text(text).font(.headline)
    }

    /// The same words `PostureFindingsPanel` shows, from the same constant.
    /// This panel REPLACES that one on an analysed running clip, so if the two
    /// ever diverge, the more clinical-sounding screen is the one that loses
    /// its note. A test pins the identity.
    static let alwaysVisibleNote = PostureFindings.alwaysVisibleNote

    /// The statement that keeps every number above it honest. Never behind a
    /// tap, on every branch of this screen — including the refusals, where the
    /// user is being told about their gait by a screen that measured nothing.
    private var notDiagnosisNote: some View {
        Text(Self.alwaysVisibleNote)
            .font(.caption2.weight(.semibold))
            .foregroundStyle(.secondary)
            .fixedSize(horizontal: false, vertical: true)
            .frame(maxWidth: .infinity, alignment: .leading)
    }

    /// FIRST, always. A claim the clip cannot resolve is not a claim.
    private func resolutionBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            header("What this clip can resolve")
            Text(s.resolutionSentence).font(.callout)
            Text(s.resolutionBreakdownSentence)
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    private func contactBlock(_ report: GaitReport, _ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            // The surviving left/right finding on this screen, so it is labelled
            // as one. It is measured from stance timing: no moment arm, no
            // muscle QP, no force model.
            header("Left vs right: time on the ground")
            // STEPS, not strides. `stance` is contacts per side, and this
            // codebase defines a stride as one leg's touchdown-to-touchdown
            // interval everywhere else (`strideSeconds`, `stridePeriodFrames`),
            // so calling the sum of both sides' contacts "strides" printed
            // roughly twice the real figure under the report's own word.
            // The two means carry their own step-to-step scatter. Printed as
            // bare means they let a one-in-four noise draw read as a finding:
            // at 5 contacts a side and this repo's own measured 11.1 % contact
            // scatter, a perfectly symmetric runner cleared the old timing-only
            // floor on 25.3 % of clips. The number beside each mean is what the
            // reader needs to see that.
            Text(String(format: "Left %.0f ms ±%.0f%% · right %.0f ms ±%.0f%% · %d steps "
                        + "measured (%d left, %d right)",
                        report.contactSeconds.left * 1000,
                        report.contactVariationPercent.left,
                        report.contactSeconds.right * 1000,
                        report.contactVariationPercent.right,
                        report.stance.left.count + report.stance.right.count,
                        report.stance.left.count, report.stance.right.count))
                .font(.callout)
            if let claim = report.asymmetryClaim {
                Text(String(format: "Contact time is %.0f%% longer on the %@ — clear of the "
                            + "±%.0f%% this clip's timing and your own step-to-step variation "
                            + "allow.",
                            abs(claim), claim > 0 ? "left" : "right",
                            report.contactClaimFloorPercent))
                    .font(.callout)
                    .foregroundStyle(.orange)
            } else {
                Text(String(format: "Left and right contact times are even to within what this "
                            + "clip can tell (measured %.1f%%, floor %.0f%% — ±%.0f%% from the "
                            + "sampling grid and your strides, ±%.0f%% from how much your own "
                            + "contact times varied step to step).",
                            abs(report.contactAsymmetryPercent),
                            report.contactClaimFloorPercent,
                            s.resolvableAsymmetryPercent,
                            report.contactSamplingUncertaintyPercent))
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    /// **What the muscle model produced, and why none of it is a claim.**
    ///
    /// This block used to be eight muscle rows: a name, `L 0.71 · R 0.55`, two
    /// bars and an orange sentence each. Every one of those is a per-muscle
    /// left/right statement, including the bare number pair — a ratio quoted to
    /// two decimals is a stronger statement than a bar length, which is why the
    /// pair survived the round that removed the bars from withheld rows and had
    /// to go with the rest.
    ///
    /// # The permanent reason comes FIRST, on every branch
    ///
    /// This used to be an `if/else` on `s.withheldReason`, and that was a defect
    /// with a lever attached. On a clip whose DATA gate failed the user saw only
    /// the data refusal — "the two contact tests disagreed on 70 % of stance
    /// frames … film a steadier, straighter run" — under a header reading
    /// "Muscle by muscle: not shown, and why". So "why" was answered with a
    /// fixable property of the clip, on a screen where NO clip, however clean,
    /// produces a muscle row: every lever in `withheldReason` was written when
    /// passing the gate produced eight rows, and since the per-muscle claim was
    /// retired, passing it produces this paragraph instead. The user re-films,
    /// the gate passes, and the answer changes to "it was never possible".
    ///
    /// So: the model limit is stated unconditionally, the clip's data failure is
    /// stated after it as a separate thing about a separate subject, and
    /// `muscleRowsUnaffectedByRefilmingSentence` scopes the lever inside it —
    /// re-filming buys the measurement, not the rows. When
    /// `perMuscleLeftRightClaimIsSupported` flips back, that sentence
    /// disappears on its own.
    @ViewBuilder
    private func loadBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            header("Muscle by muscle: not shown, and why")
            Text(s.perMuscleRetirementSentence)
                .font(.caption)
                .foregroundStyle(.secondary)
            if let reason = s.withheldReason {
                Text("Separately, about this clip's data — " + reason)
                    .font(.caption)
                    .foregroundStyle(.red)
                if let scope = s.muscleRowsUnaffectedByRefilmingSentence {
                    Text(scope)
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }
            } else {
                Text(String(format: "It ran on %d of %d claimed stance frames (%d left, "
                            + "%d right), one sample per contact — %d left contacts, %d right, "
                            + "%d muscle pairs screened.",
                            s.stanceFrameCount, s.claimedStanceFrameCount,
                            s.leftStanceFrameCount, s.rightStanceFrameCount,
                            s.leftContactCount, s.rightContactCount,
                            s.screenedComparisonCount))
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                Text(s.peakForceRegimeSentence)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
        }
    }

    /// Everything that could make the numbers above wrong, on the same screen
    /// as the numbers.
    private func honestyBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            header("What this does not measure")
            Text(s.unmodelledTermSentence).font(.caption)
            Text("Peak ground force is not measured — it is implied by contact and flight "
                 + "timing, and nothing here can contradict its size. The two checks below ask "
                 + "only whether that implied force agrees with inverse dynamics ON THE VERTICAL "
                 + "AXIS, on the frames both contact tests agreed about.")
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(s.verticalFalsifierSentence)
                .font(.caption)
                .foregroundStyle(s.residualGatePassed ? Color.secondary : Color.red)
            Text(contactAgreementLine(s))
                .font(.caption)
                .foregroundStyle(s.contactGatePassed ? Color.secondary : Color.red)
            if s.framesWithoutACleanDerivativeWindow > 0 {
                Text("\(s.framesWithoutACleanDerivativeWindow) stance frame(s) sat too close to a "
                     + "touchdown or toe-off to differentiate and were dropped.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
            if s.saturatedMuscleCount > 0 || s.flooredMuscleCount > 0 {
                // **The two counts are no longer printed, and that is the fix.**
                //
                // This line read "N muscle(s) reached full effort and M sat on
                // the resting-tone floor, out of S pairs the solver kept between
                // the two" — three defects in one sentence:
                //
                // 1. N and M are not facts about the user's muscles. STATUS
                //    measures them across a λ sweep at FIXED inputs: saturated
                //    19, 11, 22, 18, 20 and floored 219, 189, 344, 170, 282, no
                //    trend. CLAUDE.md's readings-that-lie list carries it as
                //    "it measures where OSQP stopped". Printed as a count of
                //    the reader's muscles, "20 of my muscles maxed out" is a
                //    statement about how hard they were working, off a number
                //    that changes 2× when a solver weight changes.
                // 2. The trailing disclaimer covered the ACTIVATION at the
                //    bound, not the COUNT, and the count was the part rendered
                //    as a number about a body.
                // 3. The denominator could not contain its numerators:
                //    `screenedComparisonCount` is built with
                //    `guard !saturatedBases.contains(base), !flooredBases…`, so
                //    it is disjoint from both by construction — "140 sat on the
                //    floor, out of 30 pairs" is the arithmetic that produced.
                //
                // What survives is the mechanism, which is stable, checkable and
                // the actual thing the reader needs: this QP answers with a
                // bound for most of 520 redundant muscles, and a bound is not a
                // measurement.
                Text("Most of this model's 520 muscles come back sitting on one of the "
                     + "optimiser's two bounds — full effort, or the resting-tone floor. Which "
                     + "ones do is a property of where the optimiser stopped, not of how hard "
                     + "you worked: the same frames re-solved at a different solver weight move "
                     + "that set by about a factor of two. At either bound the answer is the "
                     + "bound, not a measurement.")
                    .font(.caption)
                    .foregroundStyle(.orange)
            }
            Text(String(format: "Derivatives use a %d-tap window spanning %.0f ms against a "
                        + "shortest contact of %.0f ms — %.1f× the noise of the 9-tap window "
                        + "the still-pose path uses.",
                        s.derivativeFilterTaps,
                        s.derivativeFilterSpanMilliseconds,
                        s.shortestContactMilliseconds,
                        s.derivativeNoiseAmplification))
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }

    /// The two contact tests, with the DOUBLE-contact share named separately.
    ///
    /// "Foot height agreed with the measured contact on X %" is false for a
    /// double contact: the height agreed the claimed foot was planted, and also
    /// flagged the other one. Collapsing the two made the line describe a
    /// failure that had not happened and point at a lever that cannot fix it.
    private func contactAgreementLine(_ s: GaitLoadSummary) -> String {
        let base = String(format: "The two contact tests agreed on %.0f%% of stance frames "
                          + "(need %.0f%%) — %@.",
                          100 * s.agreementFraction,
                          100 * GaitLoadSummary.minimumContactAgreementFraction,
                          s.contactGatePassed ? "passed" : "FAILED")
        guard s.solverSawDoubleContactCount > 0 else { return base }
        return base + " \(s.solverSawDoubleContactCount) of the "
             + "\(s.contactDetectorDisagreements) disagreements are frames where the solver put "
             + "ground force under BOTH feet — the swing foot inside its 6 cm contact band, "
             + "which re-filming does not change."
    }

    @ViewBuilder
    private func flags(_ report: GaitReport) -> some View {
        if !report.flags.isEmpty {
            VStack(alignment: .leading, spacing: 2) {
                ForEach(Array(report.flags.enumerated()), id: \.offset) { _, flag in
                    Text(flag.description)
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                }
            }
        }
    }
}
