import SwiftUI

/// What a running clip is allowed to tell the user, and what it has to refuse.
///
/// Layout order is the deliberate part. The resolution line comes FIRST — before
/// any comparison — because it is the number that decides whether every other
/// line on this screen is a finding or noise, and because it is the only line
/// with an action attached ("film at a higher frame rate").
///
/// There is no newton figure anywhere on this screen. See `GaitLoadSummary`.
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
                    header("Running, but withheld")
                    ForEach(Array(report.refusals.enumerated()), id: \.offset) { _, refusal in
                        Label(refusal.description, systemImage: "exclamationmark.triangle")
                            .font(.caption)
                            .foregroundStyle(.orange)
                    }
                    Text(NimbleEngine.MotionVerdict.gaitRefused.advice)
                        .font(.caption)
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
            }
            .padding(.horizontal)
            .padding(.vertical, 10)
        }
    }

    // MARK: Blocks

    private func header(_ text: String) -> some View {
        Text(text).font(.headline)
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
            Text(String(format: "Left %.0f ms · right %.0f ms · %d strides measured",
                        report.contactSeconds.left * 1000,
                        report.contactSeconds.right * 1000,
                        report.stance.left.count + report.stance.right.count))
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

    /// The product: this muscle against that muscle, left against right, each
    /// as a normalised 0-1 load.
    private func loadBlock(_ s: GaitLoadSummary) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            header("Load during stance — relative")
            Text("Each bar is that muscle's peak effort as a fraction of what it can produce, "
                 + "measured during its own leg's contact.")
                .font(.caption)
                .foregroundStyle(.secondary)
            ForEach(s.ranked.prefix(8)) { load in
                VStack(alignment: .leading, spacing: 2) {
                    HStack {
                        Text(load.displayName).font(.caption).bold()
                        Spacer()
                        Text(String(format: "L %.2f · R %.2f", load.leftPeak, load.rightPeak))
                            .font(.caption2)
                            .foregroundStyle(.secondary)
                            .monospacedDigit()
                    }
                    HStack(spacing: 4) {
                        bar(load.leftPeak, tint: .blue)
                        bar(load.rightPeak, tint: .red)
                    }
                    Text(s.claim(for: load))
                        .font(.caption2)
                        .foregroundStyle(s.permits(differencePercent: load.differencePercent)
                                         ? .orange : .secondary)
                }
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
            Text(String(format: "Model force vs the body's own inertia: %.2f BW typical, "
                        + "%.2f BW worst (gate %.2f) — %@.",
                        s.medianForceResidualInBodyWeights,
                        s.maxForceResidualInBodyWeights,
                        NimbleEngine.maxGaitForceResidualInBodyWeights,
                        s.residualGatePassed ? "passed" : "FAILED, loads withheld"))
                .font(.caption)
                .foregroundStyle(s.residualGatePassed ? Color.secondary : Color.red)
            if s.contactDetectorDisagreements > 0 {
                Text("\(s.contactDetectorDisagreements) frame(s) where the ground-contact "
                     + "geometry disagreed with the stance detector.")
                    .font(.caption)
                    .foregroundStyle(.orange)
            }
            if s.saturatedMuscleCount > 0 {
                Text("\(s.saturatedMuscleCount) muscle(s) reached full effort, where a "
                     + "force error stops cancelling out of the ratios.")
                    .font(.caption)
                    .foregroundStyle(.orange)
            }
            Text(String(format: "Derivatives use a %d-tap window spanning %.0f ms, inside the "
                        + "shortest contact (%.0f ms).",
                        s.derivativeFilterTaps,
                        s.derivativeFilterSpanMilliseconds,
                        s.shortestContactMilliseconds))
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
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
