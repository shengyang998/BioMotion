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
/// Now `summary.arePublishable` decides the whole block, the withheld case
/// states the measurement and the lever, and the 3-D muscle overlay in
/// `OfflinePlaybackView` asks the same question.
///
/// # This screen makes ONE kind of comparison, and says which
///
/// Left against right, one muscle at a time. It does not rank muscles against
/// each other, because 66 of the model's muscles are given a straight-line path
/// where the real tendon wraps around bone and their effort numbers are
/// therefore each on a scale of their own — measured in
/// `MomentArmErrorCancellationTests`, which also measures that the same error
/// cancels out of the left/right figure. Three consequences are visible here:
/// the rows are ordered by which comparison the clip resolved best rather than
/// by load; each row's two bars are drawn to that ROW's own scale, so no bar
/// length is comparable to the row above; and `crossMuscleSentence` says both
/// things in words above the list.
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
            header("Ground contact")
            // STEPS, not strides. `stance` is contacts per side, and this
            // codebase defines a stride as one leg's touchdown-to-touchdown
            // interval everywhere else (`strideSeconds`, `stridePeriodFrames`),
            // so calling the sum of both sides' contacts "strides" printed
            // roughly twice the real figure under the report's own word.
            Text(String(format: "Left %.0f ms · right %.0f ms · %d steps measured "
                        + "(%d left, %d right)",
                        report.contactSeconds.left * 1000,
                        report.contactSeconds.right * 1000,
                        report.stance.left.count + report.stance.right.count,
                        report.stance.left.count, report.stance.right.count))
                .font(.callout)
            if let claim = report.asymmetryClaim {
                Text(String(format: "Contact time is %.0f%% longer on the %@.",
                            abs(claim), claim > 0 ? "left" : "right"))
                    .font(.callout)
                    .foregroundStyle(.orange)
            } else {
                Text(String(format: "Left and right contact times are even to within "
                            + "what this clip resolves (measured %.1f%%, floor %.0f%%).",
                            abs(report.contactAsymmetryPercent), s.resolvableAsymmetryPercent))
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    /// The product: **left against right, one muscle at a time** — or nothing at
    /// all, when the gates say so.
    @ViewBuilder
    private func loadBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            header("Left vs right, muscle by muscle")
            if let reason = s.withheldReason {
                Text(reason)
                    .font(.caption)
                    .foregroundStyle(.red)
            } else {
                Text(s.crossMuscleSentence)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                Text("Each row's two bars are that muscle's own left and right effort at "
                     + "mid-contact, drawn to that row's own scale and averaged over its leg's "
                     + "contacts.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                ForEach(s.muscles.prefix(GaitLoadSummary.displayedMuscleCount)) { load in
                    muscleRow(load, s)
                }
                Text(String(format: "From %d of %d claimed stance frames (%d left, %d right), "
                            + "one sample per contact — %d left contacts, %d right.",
                            s.stanceFrameCount, s.claimedStanceFrameCount,
                            s.leftStanceFrameCount, s.rightStanceFrameCount,
                            s.leftContactCount, s.rightContactCount))
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                Text(s.peakForceRegimeSentence)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                Text("Each comparison is a 95% one and \(GaitLoadSummary.displayedMuscleCount) "
                     + "are shown, so about one in twenty of them can read a difference that is "
                     + "not there.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
        }
    }

    /// One muscle's row.
    ///
    /// A saturated muscle gets NO bars. It used to get two full-length ones
    /// directly above a caption reading "Withheld: this muscle reached full
    /// effort" — the same show-the-number-while-saying-withheld pattern this
    /// file's header says was fixed at the clip level. The bars encode the
    /// left/right ratio, which is precisely the quantity being withheld.
    @ViewBuilder
    private func muscleRow(_ load: GaitLoadSummary.MuscleLoad, _ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(load.displayName).font(.caption).bold()
                Spacer()
                Text(String(format: "L %.2f · R %.2f", load.leftLoad, load.rightLoad))
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .monospacedDigit()
            }
            if load.isSaturated || load.isAtActivationFloor {
                Text(load.isSaturated
                     ? "Clipped at full effort — no bars drawn, because their ratio is the thing "
                     + "being withheld."
                     : "Sitting on the solver's resting-tone floor — no bars drawn, for the same "
                     + "reason.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            } else {
                let scale = load.withinMuscleScale
                HStack(spacing: 4) {
                    bar(scale > 0 ? load.leftLoad / scale : 0, tint: .blue)
                    bar(scale > 0 ? load.rightLoad / scale : 0, tint: .red)
                }
            }
            Text(s.claim(for: load))
                .font(.caption2)
                .foregroundStyle(s.permits(load) ? .orange : .secondary)
            if !load.pathIsModelled {
                Text("This muscle's path is modelled as a straight line where it really wraps "
                     + "around bone, so its 0-1 numbers are on a scale of their own — compare "
                     + "them left to right, not to the row above.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private func bar(_ value: Double, tint: Color) -> some View {
        GeometryReader { geo in
            ZStack(alignment: .leading) {
                Capsule().fill(tint.opacity(0.15))
                Capsule()
                    .fill(tint)
                    .frame(width: geo.size.width * min(max(value, 0), 1))
            }
        }
        .frame(height: 6)
    }

    /// Everything that could make the numbers above wrong, on the same screen
    /// as the numbers.
    private func honestyBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            header("What this does not measure")
            Text(s.unmodelledTermSentence).font(.caption)
            Text("Peak ground force is not measured — it is implied by contact and flight "
                 + "timing, and nothing here can contradict its size. Left/right ratios are "
                 + "shown because a size error cancels out of them; the two checks below test "
                 + "that cancellation ON THE VERTICAL AXIS ONLY.")
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
                Text("\(s.saturatedMuscleCount) muscle(s) reached full effort and "
                     + "\(s.flooredMuscleCount) sat on the resting-tone floor; their left/right "
                     + "comparisons are withheld, because a force error stops cancelling at "
                     + "either bound.")
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
