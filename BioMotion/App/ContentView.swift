import SwiftUI

struct ContentView: View {
    @StateObject private var bodyTracking = BodyTrackingSession()
    @StateObject private var recorder = MotionRecorder()
    @StateObject private var nimble = NimbleEngine()
    @State private var showExportSheet = false
    @State private var exportURLs: [URL] = []
    @State private var showIKPanel = false
    @State private var showCalibration = true
    @State private var showCharts = false
    /// The 3-D muscle ANATOMY layer. It draws where the muscles are; it does
    /// not encode effort — see `MuscleOverlay`.
    @State private var showAnatomyOverlay = true
    @State private var showOfflineImport = false

    private var liveAnatomyPresentation: LiveAnatomyPresentation {
        LiveAnatomyPresentation(
            surface: .tracking,
            isTracking: bodyTracking.isTracking,
            hasCurrentFrame: bodyTracking.currentFrame != nil,
            isEnabled: showAnatomyOverlay
        )
    }

    var body: some View {
        Group {
            if showCalibration {
                CalibrationView(
                    bodyTracking: bodyTracking,
                    nimble: nimble,
                    onComplete: {
                        withAnimation { showCalibration = false }
                    }
                )
            } else if showCharts {
                SessionChartsView(
                    nimble: nimble,
                    recorder: recorder,
                    onExport: { exportAll() },
                    onDismiss: { showCharts = false }
                )
            } else {
                trackingView
            }
        }
        .onAppear {
            bodyTracking.start()
            nimble.loadBundledModel()
        }
        .onChange(of: bodyTracking.currentFrame?.frameNumber) { _, _ in
            guard let frame = bodyTracking.currentFrame else { return }
            // Offline analysis drives the SAME NimbleEngine — its SG filters and
            // the process-wide skeleton. A live frame arriving mid-session does
            // not just waste power, it interleaves a second motion into the
            // filters the offline run is reading.
            guard !showCalibration && !showCharts && !showOfflineImport else { return }
            recorder.recordFrame(frame)
            nimble.processFrame(frame)
        }
        .onChange(of: bodyTracking.isTracking) { _, isTracking in
            if !isTracking {
                nimble.resetRealtimeState()
            }
        }
        // Charts shown via export button, not auto-transition
        .sheet(isPresented: $showExportSheet) {
            ShareSheet(items: exportURLs)
        }
        .onChange(of: showOfflineImport) { _, presenting in
            if presenting {
                bodyTracking.pause()
                // Drop the live filter warm-up so the offline session starts from
                // an empty window rather than inheriting live samples.
                nimble.resetRealtimeState()
            } else {
                bodyTracking.start()
            }
        }
        .sheet(isPresented: $showOfflineImport) {
            OfflineImportView(nimble: nimble, onDismiss: { showOfflineImport = false })
        }
    }

    private var trackingView: some View {
        ZStack {
            // Camera feed
            SkeletonARView(
                session: bodyTracking.arSession,
                currentFrame: $bodyTracking.currentFrame,
                isTracking: bodyTracking.isTracking,
                anatomyPresentation: liveAnatomyPresentation
            )
            .ignoresSafeArea()

            // Single VStack overlay: top → spacer → bottom
            VStack(spacing: 0) {
                // === TOP BAR ===
                HStack {
                    StatusBadge(text: bodyTracking.trackingMessage, isActive: bodyTracking.isTracking)
                    if nimble.isModelLoaded {
                        StatusBadge(text: String(format: "IK %.1fms", nimble.ikSolveTimeMs), isActive: true)
                    }
                    Spacer()
                    Button {
                        showOfflineImport = true
                    } label: {
                        Image(systemName: "photo.on.rectangle")
                            .foregroundStyle(.white)
                            .padding(8)
                            .background(.black.opacity(0.6), in: Circle())
                    }
                    if let frame = bodyTracking.currentFrame {
                        let tracked = frame.joints.filter(\.isTracked).count
                        StatusBadge(text: "\(tracked)/\(frame.joints.count)", isActive: tracked > 0)
                    }
                }
                .padding(.horizontal, 12)
                .padding(.top, 4)

                // Accuracy diagnostics: two rows of small pills.
                // Row 1 — IK residual, max joint torque per kg, model mass.
                // Row 2 — left/right foot load (fraction of body weight),
                //         GRF root residual. Target ranges after full overhaul:
                //   residual ≈ 0.01–0.03 m (ARKit-limited floor)
                //   |τ|/m    ≈ 1–3 Nm/kg for walking/squat (physiological)
                //   L+R load ≈ 1.0 ± 0.1 in stance (weight supported by feet)
                //   root res ≈ < 0.5 Nm/kg (GRF consistent with kinematics)
                if nimble.isModelLoaded && bodyTracking.isTracking {
                    VStack(alignment: .leading, spacing: 2) {
                        HStack(spacing: 6) {
                            AccuracyBadge(
                                label: "residual",
                                // Millimetres, because this is a length. It used
                                // to print the solver's m² LOSS as "%.3f m" with
                                // a green cut at 0.05 — a squared quantity shown
                                // as a distance. `ikMarkerResidualMeters` is now
                                // the true per-marker RMS. The 20 mm line is the
                                // measured standing fit (0.03 mm) versus the
                                // dancer's model mismatch (21 mm), not a target.
                                value: String(format: "%.0f mm", nimble.ikMarkerResidualMeters * 1000),
                                good: nimble.ikMarkerResidualMeters < 0.020
                            )
                            AccuracyBadge(
                                label: "max |τ|/m",
                                value: String(format: "%.1f Nm/kg", nimble.maxTorquePerKg),
                                good: nimble.maxTorquePerKg < 5.0
                            )
                            if nimble.totalMassKg > 0 {
                                AccuracyBadge(
                                    label: "mass",
                                    value: String(format: "%.0f kg", nimble.totalMassKg),
                                    good: true
                                )
                            }
                            Spacer()
                        }
                        HStack(spacing: 6) {
                            // **This badge used to print the LEFT/RIGHT SPLIT,
                            // and it was the app's most-used screen making the
                            // exact claim the offline path spent four rounds
                            // retiring — with no caption, no floor and nothing
                            // validating it.**
                            //
                            // It is a diagnostic now, and it shows the quantity
                            // its own indicator has always checked: the SUM.
                            // `good` was `abs(total - 1.0) < 0.3`, keyed to the
                            // sum alone, while the value read "0.62|0.38".
                            //
                            // There is no discipline that could have rescued the
                            // split instead. `NimbleBridge.mm:1499` seeds the
                            // near-CoP solver with a hardcoded 50/50 wrench
                            // guess when both feet are down, and the solver's
                            // constraint fixes ΣF exactly while leaving each
                            // foot's CoP free inside its own polygon — so the
                            // split is statically indeterminate (STATUS sizes it
                            // at ±18 pp with a perfectly known CoM against a
                            // ~10 pp clinical threshold) AND anchored to a
                            // prior. It is an artifact with a plausible shape,
                            // which is the worst kind of number to draw.
                            let totalLoad = nimble.leftFootLoadFraction + nimble.rightFootLoadFraction
                            AccuracyBadge(
                                label: "GRF sum",
                                value: String(format: "%.2f BW", totalLoad),
                                good: abs(totalLoad - 1.0) < 0.3 || totalLoad < 0.1
                            )
                            // N/kg, not Nm/kg: this is now a linear-momentum
                            // residual. It checks that the contact wrenches were
                            // read back in the right frame, so a correct pipeline
                            // reports ~0 whether or not the pose is balanced —
                            // green here means "consistent", never "balanced".
                            AccuracyBadge(
                                label: "frame chk",
                                value: String(format: "%.2f N/kg", nimble.rootResidualPerKg),
                                good: nimble.rootResidualPerKg < 0.5
                            )
                            Spacer()
                        }
                        // Same `if` as the badges above it, deliberately: a
                        // number and the sentence that scopes it must not be
                        // gated on different conditions.
                        Text(NimbleEngine.footLoadSplitIsNotMeasuredNote)
                            .font(.caption2)
                            .foregroundStyle(.white.opacity(0.75))
                            .fixedSize(horizontal: false, vertical: true)
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                    .padding(.horizontal, 12)
                    .padding(.top, 2)
                }

                // === PUSH EVERYTHING ELSE DOWN ===
                Spacer()

                // === BOTTOM SECTION ===

                // Data panels (only when tracking)
                //
                // **The muscle bar chart used to be here, and it made the claim
                // the 3-D overlay was making in colour.** Twelve named muscles,
                // bar height ∝ activation, a blue→red colour cut and "71 %"
                // under each — a cross-muscle ranking AND an absolute effort
                // figure, on numbers whose per-muscle scale is unknown. The
                // reason given at the time was the straight-line paths (66 of
                // this model's muscles), and those are wrapped since 2026-08-08;
                // what is left is the reason that never depended on them —
                // nothing puts two muscles' activations on one scale. The QP's
                // own termination slack was a second reason for one commit (a
                // median of 14.88 pp at fixed geometry); `scaling = 0` plus
                // `polishing = 1` took it to 4.4994e-05 pp on 2026-08-09, and the
                // absent common scale is the one that cannot be solved harder.
                // Reading soleus 0.71 against vastus 0.34 is exactly what
                // `MomentArmErrorCancellationTests` shows this model cannot
                // support, and the offline panel retired the same comparison on
                // 2026-08-08.
                //
                // What replaces it is the absence, stated. The engineering
                // diagnostics above (marker residual, |τ|/kg, foot-load
                // fractions, frame check) are unchanged: they are labelled with
                // their units and none of them is a per-muscle claim.
                if liveAnatomyPresentation.anatomyIsPresented {
                    Text(MuscleOverlay.anatomyOnlyNote)
                        .font(.caption2)
                        .foregroundStyle(.white.opacity(0.85))
                        .fixedSize(horizontal: false, vertical: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .padding(8)
                        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
                        .padding(.horizontal, 12)
                }
                if showIKPanel, let ik = nimble.lastIKResult {
                    IKReadoutPanel(ikResult: ik, idResult: nimble.lastIDResult).padding(.horizontal, 12)
                } else if bodyTracking.isTracking, let frame = bodyTracking.currentFrame {
                    JointReadoutPanel(frame: frame).padding(.horizontal, 12)
                }

                // Toggle buttons
                if (nimble.isModelLoaded && bodyTracking.isTracking)
                    || liveAnatomyPresentation.showsControl {
                    HStack(spacing: 8) {
                        if nimble.isModelLoaded && bodyTracking.isTracking {
                            Button { withAnimation { showIKPanel.toggle() } } label: {
                                Text(showIKPanel ? "Positions" : "IK/ID")
                                    .font(.caption2).foregroundStyle(.white)
                                    .padding(.horizontal, 10).padding(.vertical, 4)
                                    .background(.black.opacity(0.5), in: Capsule())
                            }
                        }
                        if liveAnatomyPresentation.showsControl {
                            // "Muscles ON" promised a muscle reading. The layer
                            // is anatomy — where they are — so the control says so.
                            Button { showAnatomyOverlay.toggle() } label: {
                                HStack(spacing: 4) {
                                    Image(systemName: "figure.stand")
                                    Text(showAnatomyOverlay ? "Anatomy ON" : "Anatomy OFF")
                                }
                                .font(.caption2)
                                .foregroundStyle(showAnatomyOverlay ? .green : .gray)
                                .padding(.horizontal, 10).padding(.vertical, 4)
                                .background(.black.opacity(0.5), in: Capsule())
                            }
                        }
                    }
                    .padding(.top, 4)
                }

                // === RECORD BAR (always at very bottom) ===
                HStack {
                    if recorder.isRecording {
                        HStack(spacing: 6) {
                            Circle().fill(.red).frame(width: 8, height: 8)
                            Text(formatDuration(recorder.duration)).monospacedDigit()
                            Text("\(recorder.recordedFrameCount)f")
                        }
                        .font(.caption).foregroundStyle(.white)
                        .padding(.horizontal, 10).padding(.vertical, 6)
                        .background(.black.opacity(0.6), in: Capsule())
                    } else if recorder.hasRecording {
                        Button { exportAll() } label: {
                            Label("Export", systemImage: "square.and.arrow.up")
                                .font(.caption).foregroundStyle(.white)
                                .padding(.horizontal, 10).padding(.vertical, 6)
                                .background(.blue.opacity(0.8), in: Capsule())
                        }
                    } else {
                        Color.clear.frame(width: 80, height: 1)
                    }

                    Spacer()

                    Button {
                        if recorder.isRecording {
                            recorder.stopRecording()
                            nimble.stopRecordingResults()
                        } else {
                            recorder.startRecording()
                            nimble.startRecordingResults()
                        }
                    } label: {
                        ZStack {
                            Circle().strokeBorder(.white, lineWidth: 3).frame(width: 64, height: 64)
                            if recorder.isRecording {
                                RoundedRectangle(cornerRadius: 4).fill(.red).frame(width: 24, height: 24)
                            } else {
                                Circle().fill(.red).frame(width: 52, height: 52)
                            }
                        }
                    }

                    Spacer()
                    Color.clear.frame(width: 80, height: 1)
                }
                .padding(.horizontal, 16)
                .padding(.top, 8)
                .padding(.bottom, 30)
            }
        }
    }

    private func exportAll() {
        var urls: [URL] = []
        var errors: [String] = []

        // Export .trc (marker positions)
        if recorder.hasRecording {
            let trcExporter = TRCExporter(frames: recorder.frames)
            do {
                let url = try trcExporter.export()
                urls.append(url)
            } catch {
                errors.append("TRC: \(error.localizedDescription)")
            }
        } else {
            errors.append("No recording data")
        }

        // Export .mot (joint angles from IK)
        do {
            let url = try nimble.exportMOT()
            urls.append(url)
        } catch {
            errors.append("MOT: no IK data")
        }

        // Export .sto (joint torques from ID)
        do {
            let url = try nimble.exportSTO()
            urls.append(url)
        } catch {
            errors.append("STO: no ID data")
        }

        if !urls.isEmpty {
            exportURLs = urls
            showExportSheet = true
        } else {
            // Nothing to export — show alert with reason
            // For now, export a summary text file so the share sheet isn't empty
            let summary = """
            BioMotion Export — No data available

            Reasons:
            \(errors.joined(separator: "\n"))

            Tips:
            - Make sure body tracking is active (green status)
            - Record for at least a few seconds
            - The Nimble model must load successfully
            """
            let url = FileManager.default.temporaryDirectory.appendingPathComponent("BioMotion_export_info.txt")
            try? summary.write(to: url, atomically: true, encoding: .utf8)
            exportURLs = [url]
            showExportSheet = true
        }
    }

    private func formatDuration(_ duration: TimeInterval) -> String {
        let minutes = Int(duration) / 60
        let seconds = Int(duration) % 60
        let fraction = Int((duration - Double(Int(duration))) * 10)
        return String(format: "%d:%02d.%d", minutes, seconds, fraction)
    }
}

// MARK: - Supporting Views

struct StatusBadge: View {
    let text: String
    let isActive: Bool

    var body: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(isActive ? .green : .orange)
                .frame(width: 8, height: 8)
            Text(text)
                .font(.caption)
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(.black.opacity(0.6), in: Capsule())
    }
}

/// Compact label+value pill used for precision diagnostics in the HUD.
struct AccuracyBadge: View {
    let label: String
    let value: String
    let good: Bool

    var body: some View {
        HStack(spacing: 4) {
            Text(label).font(.system(size: 9, weight: .regular, design: .monospaced))
                .foregroundStyle(.white.opacity(0.7))
            Text(value).font(.system(size: 10, weight: .semibold, design: .monospaced))
                .foregroundStyle(good ? Color.green : Color.orange)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 3)
        .background(.black.opacity(0.55), in: Capsule())
    }
}

struct JointReadoutPanel: View {
    let frame: BodyFrame

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text("Joint Positions (m)")
                .font(.caption2.bold())
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 12) {
                    ForEach(frame.joints.filter(\.isTracked)) { joint in
                        VStack(alignment: .leading, spacing: 2) {
                            Text(joint.name)
                                .font(.caption2.bold())
                            Text(String(format: "%.2f, %.2f, %.2f",
                                        joint.worldPosition.x,
                                        joint.worldPosition.y,
                                        joint.worldPosition.z))
                                .font(.system(.caption2, design: .monospaced))
                        }
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(8)
        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }
}

struct IKReadoutPanel: View {
    let ikResult: NimbleEngine.IKOutput
    let idResult: NimbleEngine.IDOutput?

    // Show a curated set of important DOFs
    private let keyDOFs = [
        "hip_flexion_r", "hip_flexion_l",
        "knee_angle_r", "knee_angle_l",
        "ankle_angle_r", "ankle_angle_l",
        "lumbar_extension",
    ]

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text("IK Joint Angles")
                    .font(.caption2.bold())
                Spacer()
                Text(String(format: "marker RMS: %.1f mm", ikResult.markerRMSMeters * 1000))
                    .font(.system(.caption2, design: .monospaced))
            }
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 12) {
                    ForEach(keyDOFs, id: \.self) { dof in
                        if let angle = ikResult.jointAngles[dof] {
                            VStack(alignment: .leading, spacing: 2) {
                                Text(shortName(dof))
                                    .font(.caption2.bold())
                                Text(String(format: "%.1f\u{00B0}", angle * 180.0 / .pi))
                                    .font(.system(.caption2, design: .monospaced))
                                if let torque = idResult?.jointTorques[dof] {
                                    Text(String(format: "%.1f Nm", torque))
                                        .font(.system(.caption2, design: .monospaced))
                                        .foregroundStyle(.yellow)
                                }
                            }
                        }
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(8)
        .background(.black.opacity(0.5), in: RoundedRectangle(cornerRadius: 8))
    }

    private func shortName(_ dof: String) -> String {
        dof.replacingOccurrences(of: "_r", with: " R")
           .replacingOccurrences(of: "_l", with: " L")
           .replacingOccurrences(of: "_", with: " ")
           .capitalized
    }
}

// `MuscleActivationBar` stood here until 2026-08-08. See the comment at its
// call site in `trackingView` for why a chart of twelve muscles' activations,
// ranked against each other and labelled in per cent, is not a reading this
// model can produce.

/// UIKit share sheet wrapper.
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]

    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }

    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
