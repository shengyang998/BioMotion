import SwiftUI

/// Product presentation for a running clip.
///
/// The input is `GaitTimingReport`, a detached value projection containing only
/// timestamp-derived contact timing, its uncertainty, refusals, and flags. The
/// historical force hypothesis, dynamics plan, native residual, and
/// `GaitLoadSummary` have no initializer path into this view.
struct GaitReportPanel: View {
    let outcome: OfflineResultStore.GaitOutcome
    let hasValidatedFootContactSupport: Bool?

    init(outcome: OfflineResultStore.GaitOutcome,
         hasValidatedFootContactSupport: Bool? = nil) {
        self.outcome = outcome
        self.hasValidatedFootContactSupport = hasValidatedFootContactSupport
    }

    /// Pure policy value used by `body` and pinned without introspecting a
    /// SwiftUI tree.
    struct AnalysedPresentation {
        let timing: GaitTimingSummary
        let contactSupportMessage: String?
        let showsResolution = true
        let showsContactTime = true
        let showsFlags = true
    }

    static func analysedPresentation(
        report: GaitTimingReport,
        hasValidatedFootContactSupport: Bool? = nil
    ) -> AnalysedPresentation {
        AnalysedPresentation(
            timing: report.timing,
            contactSupportMessage: analysedContactSupportMessage(
                hasValidatedFootContactSupport: hasValidatedFootContactSupport))
    }

    static let contactSupportUnavailableMessage =
        "Joint torque, ground force, centre of pressure, muscle effort and gait-load values "
        + "are not available because this model and solver have no validated foot-support "
        + "mechanics. This is a permanent model limitation; refilming cannot enable them."

    static func analysedContactSupportMessage(
        hasValidatedFootContactSupport: Bool?
    ) -> String? {
        hasValidatedFootContactSupport == false
            ? "The contact-time result above is complete and independent of load mechanics."
            : nil
    }

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
                    ForEach(Array(report.refusals.enumerated()), id: \.offset) { _, refusal in
                        VStack(alignment: .leading, spacing: 2) {
                            Label(refusal.description,
                                  systemImage: "exclamationmark.triangle")
                                .font(.caption)
                                .foregroundStyle(.orange)
                            Text(refusal.advice(framesPerSecond: report.framesPerSecond))
                                .font(.caption2)
                                .foregroundStyle(.secondary)
                        }
                    }
                    flags(report)

                case .analysed(let report):
                    let presentation = Self.analysedPresentation(
                        report: report,
                        hasValidatedFootContactSupport: hasValidatedFootContactSupport)
                    if presentation.showsResolution {
                        resolutionBlock(presentation.timing)
                        Divider()
                    }
                    if presentation.showsContactTime {
                        contactBlock(report, presentation.timing)
                    }
                    if let message = presentation.contactSupportMessage {
                        timingIndependenceBlock(message)
                    }
                    if presentation.showsFlags {
                        flags(report)
                    }
                }
                notDiagnosisNote
            }
            .padding(.horizontal)
            .padding(.vertical, 10)
        }
    }

    // MARK: - Blocks

    private func header(_ text: String) -> some View {
        Text(text).font(.headline)
    }

    static let alwaysVisibleNote = PostureFindings.alwaysVisibleNote

    private var notDiagnosisNote: some View {
        Text(Self.alwaysVisibleNote)
            .font(.caption2.weight(.semibold))
            .foregroundStyle(.secondary)
            .fixedSize(horizontal: false, vertical: true)
            .frame(maxWidth: .infinity, alignment: .leading)
    }

    private func resolutionBlock(_ summary: GaitTimingSummary) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            header("What this clip can resolve")
            Text(summary.resolutionSentence).font(.callout)
            Text(summary.resolutionBreakdownSentence)
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    private func contactBlock(
        _ report: GaitTimingReport,
        _ summary: GaitTimingSummary
    ) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            header("Left vs right: time on the ground")
            Text(String(
                format: "Left %.0f ms ±%.0f%% · right %.0f ms ±%.0f%% · %d steps "
                    + "measured (%d left, %d right)",
                report.contactSeconds.left * 1000,
                report.contactVariationPercent.left,
                report.contactSeconds.right * 1000,
                report.contactVariationPercent.right,
                report.contactCounts.left + report.contactCounts.right,
                report.contactCounts.left,
                report.contactCounts.right))
                .font(.callout)

            if let claim = report.asymmetryClaim {
                Text(String(
                    format: "Contact time is %.0f%% longer on the %@ — clear of the "
                        + "±%.0f%% this clip's timing and your own step-to-step variation allow.",
                    abs(claim), claim > 0 ? "left" : "right",
                    report.contactClaimFloorPercent))
                    .font(.callout)
                    .foregroundStyle(.orange)
            } else {
                Text(String(
                    format: "Left and right contact times are even to within what this clip "
                        + "can tell (measured %.1f%%, floor %.0f%% — ±%.0f%% from the "
                        + "sampling grid and your strides, ±%.0f%% from how much your own "
                        + "contact times varied step to step).",
                    abs(report.contactAsymmetryPercent),
                    report.contactClaimFloorPercent,
                    summary.resolvableAsymmetryPercent,
                    report.contactSamplingUncertaintyPercent))
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private func timingIndependenceBlock(_ message: String) -> some View {
        Text(message)
            .font(.caption)
            .foregroundStyle(.secondary)
    }

    @ViewBuilder
    private func flags(_ report: GaitTimingReport) -> some View {
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
