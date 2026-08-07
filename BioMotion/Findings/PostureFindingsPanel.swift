import SwiftUI

/// The findings panel: what the app actually tells the user about their posture.
///
/// Layout rules, in priority order:
///  1. The biggest deviation reads first — `PostureReport.findings` arrives
///     already ranked, and this view never reorders it.
///  2. Every row shows the NUMBER and the two landmarks it was measured
///     between, so the reader can check it against the photo on screen. No row
///     is ever just a verdict.
///  3. Nothing is silently dropped. Findings the view cannot support are listed
///     under "Not measurable from this view" with the reason; findings that came
///     back near zero are listed as "no measurable deviation".
///  4. No colour coding by severity. Colour would imply a normal range, and no
///     source for one is established here.
struct PostureFindingsPanel: View {
    let report: PostureReport

    @State private var showsNotes = false

    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            header
            Divider()
            ScrollView {
                VStack(alignment: .leading, spacing: 14) {
                    if report.findings.isEmpty {
                        emptyState
                    } else {
                        ForEach(report.findings) { finding in
                            row(finding)
                        }
                    }

                    if !report.negligible.isEmpty {
                        secondarySection(
                            title: "No measurable deviation",
                            lines: report.negligible.map { "\($0.title) — \($0.formattedValue)" })
                    }

                    if !report.suppressed.isEmpty {
                        secondarySection(
                            title: "Not measurable from this view",
                            lines: report.suppressed.map { "\($0.title) — \($0.reason)" })
                    }

                    notes
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 12)
            }
        }
        .background(.thinMaterial)
    }

    // MARK: - Header

    private var header: some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack(alignment: .firstTextBaseline) {
                Text("Posture")
                    .font(.headline)
                Spacer()
                Text(report.view.label)
                    .font(.caption.weight(.semibold))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(.quaternary, in: Capsule())
            }
            // The view assessment is stated up front rather than buried: which
            // findings a single photo can support is decided by the subject's
            // orientation to the camera, and that is the single biggest source
            // of wrong numbers in this layer.
            Text(report.view.summary)
                .font(.caption2)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.horizontal, 14)
        .padding(.top, 10)
        .padding(.bottom, 8)
    }

    // MARK: - Rows

    private func row(_ finding: PostureFinding) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack(alignment: .firstTextBaseline, spacing: 8) {
                Text(finding.title)
                    .font(.subheadline.weight(.semibold))
                Spacer(minLength: 8)
                Text(finding.formattedValue)
                    .font(.subheadline.weight(.semibold).monospacedDigit())
            }
            Text(finding.sideMeaning)
                .font(.caption)
                .foregroundStyle(.primary)
            Text(finding.measuredBetween)
                .font(.caption2)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
            Text(finding.projectionNote)
                .font(.caption2)
                .foregroundStyle(.tertiary)
            if let caveat = finding.caveat {
                Text(caveat)
                    .font(.caption2)
                    .foregroundStyle(.orange)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var emptyState: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(report.hasAnything
                 ? "No deviation large enough to report from this frame."
                 : "No posture measurement was possible for this frame.")
                .font(.subheadline)
                .foregroundStyle(.secondary)

            // Measured on a real prediction: an obliquely-viewed subject
            // suppresses ALL NINE findings, at depth fractions of 62% and 80%.
            // Listing nine "cannot measure" rows and stopping there tells the
            // user nothing they can act on. Every suppression here is a property
            // of the camera angle, and the camera angle is the one thing they
            // can trivially change, so say so.
            if !report.suppressed.isEmpty && report.findings.isEmpty {
                Text(retakeAdvice)
                    .font(.subheadline)
                    .foregroundStyle(.primary)
            }
        }
    }

    /// What to change about the photo. Which findings are recoverable depends on
    /// which way the subject was facing, so the advice names the view that would
    /// unlock the most, rather than giving one generic instruction.
    private var retakeAdvice: String {
        switch report.view.orientation {
        case .oblique, .undetermined:
            return "The subject is at an angle to the camera, so nothing here can be "
                 + "measured reliably. Retake square to the camera for shoulder and "
                 + "weight-shift measurements, or square to the side for head and "
                 + "trunk-lean measurements."
        case .sagittal:
            return "This is a side view. Head, trunk-lean and upper-back measurements "
                 + "need the subject closer to fully side-on; shoulder asymmetry and "
                 + "weight shift need a front-on photo instead."
        case .frontal:
            return "This is a front view. Shoulder and weight-shift measurements need "
                 + "the subject closer to fully square on; head position and trunk lean "
                 + "need a side-on photo instead."
        }
    }

    private func secondarySection(title: String, lines: [String]) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption.weight(.semibold))
                .foregroundStyle(.secondary)
            ForEach(Array(lines.enumerated()), id: \.offset) { _, line in
                Text(line)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
        }
        .padding(.top, 2)
    }

    // MARK: - Notes

    private var notes: some View {
        VStack(alignment: .leading, spacing: 6) {
            // Never behind a tap: this is the statement that keeps every number
            // above it honest.
            Text(PostureFindings.alwaysVisibleNote)
                .font(.caption2.weight(.semibold))
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)
                .frame(maxWidth: .infinity, alignment: .leading)

            Button {
                showsNotes.toggle()
            } label: {
                HStack(spacing: 4) {
                    Image(systemName: showsNotes ? "chevron.down" : "chevron.right")
                    Text("How these numbers are made")
                }
                .font(.caption2.weight(.semibold))
            }
            .buttonStyle(.plain)
            .foregroundStyle(.secondary)

            if showsNotes {
                ForEach(Array(PostureFindings.methodNotes.enumerated()), id: \.offset) { _, note in
                    Text("• " + note)
                        .font(.caption2)
                        .foregroundStyle(.secondary)
                        .fixedSize(horizontal: false, vertical: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
            }
        }
        .padding(.top, 6)
    }
}
