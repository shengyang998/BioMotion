import SwiftUI
import PhotosUI
import UniformTypeIdentifiers

/// Entry screen for offline import: pick a photo or video from the library,
/// choose a sampling rate, run the batch pipeline, then hand off to
/// `OfflinePlaybackView`.
///
/// No `NSPhotoLibraryUsageDescription` is required for this flow: `PhotosPicker`
/// (PhotosUI, iOS 16+) runs the picker UI out-of-process, like a share sheet, and
/// only grants this app access to the specific item(s) the user selects — Apple
/// explicitly documents that it does not require photo library permission. This
/// applies to both images and videos. (Flagged explicitly because the task brief
/// for this file anticipated a project.yml/Info.plist diff might be needed here —
/// having checked, it is not, for this reason.)
struct OfflineImportView: View {
    @ObservedObject var nimble: NimbleEngine
    let onDismiss: () -> Void

    /// The transfer belongs to iOS rather than this sheet. Observing the shared
    /// store keeps its system-reported state visible while selection, run, and
    /// Close remain independent lifecycle controls.
    @ObservedObject private var modelStore: AssetPackModelStore
    @StateObject private var runner: OfflineSessionRunner
    @State private var pickerItem: PhotosPickerItem?
    @State private var selection = OfflineImportSelectionState()
    @State private var selectionLoadTask: Task<Void, Never>?
    @State private var isSingleFrameMode = false
    @State private var useNativeWindow = true
    @State private var fps: Double = OfflineImportView.defaultFPS
    @State private var showPlayback = false
    @State private var selfTest: String?
    @State private var selfTestRunning = false

    static let defaultFPS = 2.0

    init(nimble: NimbleEngine, onDismiss: @escaping () -> Void) {
        self.nimble = nimble
        self.onDismiss = onDismiss
        _modelStore = ObservedObject(wrappedValue: AssetPackModelStore.shared)
        _runner = StateObject(wrappedValue: OfflineSessionRunner(nimble: nimble))
    }

    var body: some View {
        NavigationStack {
            Form {
                pickerSection
                modelAvailabilitySection
                if selection.selectedPhoto != nil || selection.selectedVideo != nil {
                    samplingSection
                    runSection
                }
                if case .failed(let message) = runner.phase {
                    Section {
                    // Runs the model on a synthetic tensor any machine can
                    // reproduce bit-exactly. Matching input checksums with
                    // differing output checksums proves the two Core ML backends
                    // compute different things from identical bytes — with no
                    // decode, Vision or warp left in the chain to blame.
                    Button {
                        selfTestRunning = true
                        Task {
                            do {
                                let r = try await SAM3DPoseEstimator.backendSelfTest()
                                selfTest = String(format: "in  %016llx\nout %016llx", r.input, r.output)
                            } catch {
                                selfTest = "failed: \(error.localizedDescription)"
                            }
                            selfTestRunning = false
                        }
                    } label: {
                        HStack {
                            Text(selfTestRunning ? "Running model self-test…" : "Run model self-test")
                            if selfTestRunning { Spacer(); ProgressView() }
                        }
                    }
                    .disabled(selfTestRunning)
                    if let selfTest {
                        Text(selfTest)
                            .font(.system(size: 11, design: .monospaced))
                            .textSelection(.enabled)
                    }
                } header: {
                    Text("Diagnostics")
                } footer: {
                    Text("Compares this device's Core ML backend against the reference machine on a fixed synthetic input. Downloads the model if it isn't present yet.")
                }

                Section("Error") {
                        Text(message).foregroundStyle(.red)
                    }
                }
            }
            .navigationTitle("Import Clip")
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Close", action: close)
                }
            }
            .interactiveDismissDisabled(isRunning)
            .onChange(of: pickerItem) { _, newItem in
                guard let newItem else { return }
                // Advance the generation before cancelling A. If A handles its
                // cancellation after B starts, its stale token cannot clear B's
                // loading state or replace the last usable selection.
                let generation = selection.beginLoading()
                selectionLoadTask?.cancel()
                selectionLoadTask = Task {
                    let isVideo = newItem.supportedContentTypes.contains {
                        $0.conforms(to: .movie) || $0.conforms(to: .video)
                    }
                    do {
                        if isVideo {
                            let movie = try await newItem.loadTransferable(
                                type: PickedMovie.self
                            )
                            try Task.checkCancellation()
                            guard let movie else {
                                _ = selection.fail(
                                    "Couldn't load the selected video.",
                                    generation: generation
                                )
                                return
                            }
                            if selection.commit(video: movie.owner, generation: generation) {
                                isSingleFrameMode = false
                            }
                        } else {
                            let data = try await newItem.loadTransferable(type: Data.self)
                            try Task.checkCancellation()
                            guard let data else {
                                _ = selection.fail(
                                    "Couldn't load the selected photo.",
                                    generation: generation
                                )
                                return
                            }
                            guard let image = UIImage(data: data) else {
                                _ = selection.fail(
                                    "Couldn't load the selected photo.",
                                    generation: generation
                                )
                                return
                            }
                            _ = selection.commit(photo: image, generation: generation)
                        }
                    } catch is CancellationError {
                        _ = selection.cancel(generation: generation)
                    } catch {
                        _ = selection.fail(
                            "Couldn't load the selection: \(error.localizedDescription)",
                            generation: generation
                        )
                    }
                }
            }
            .onChange(of: runner.phase) { _, newPhase in
                if case .finished(let processed, _, let cancelled) = newPhase, processed > 0, !cancelled {
                    showPlayback = true
                }
            }
            .navigationDestination(isPresented: $showPlayback) {
                OfflinePlaybackView(resultStore: runner.resultStore, onDone: onDismiss)
            }
            .onDisappear {
                // Navigation to playback hides this form too; that transition
                // owns the completed result store and must not trigger a reset.
                guard !showPlayback else { return }
                selectionLoadTask?.cancel()
                runner.cancel()
            }
        }
    }

    // MARK: - Sections

    @ViewBuilder
    private var modelAvailabilitySection: some View {
        switch modelStore.state {
        case .checking:
            Section("Pose Model") {
                HStack {
                    ProgressView()
                    Text("Checking pose model availability…")
                }
                if let message = modelStore.state.message {
                    Text(message)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        case .downloading(let progress):
            Section("Pose Model") {
                ProgressView(value: progress.fraction) {
                    Text("Downloading pose model…")
                } currentValueLabel: {
                    Text("\(Int((progress.fraction * 100).rounded()))%")
                }
                if let message = modelStore.state.message {
                    Text(message)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        case .paused(let progress):
            Section("Pose Model") {
                if let progress {
                    ProgressView(value: progress.fraction) {
                        Text("Pose model download paused")
                    } currentValueLabel: {
                        Text("\(Int((progress.fraction * 100).rounded()))%")
                    }
                } else {
                    Label("Pose model download paused", systemImage: "pause.circle")
                }
                if let message = modelStore.state.message {
                    Text(message)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        case .unavailable(let message):
            Section("Pose Model") {
                Text(message)
                    .foregroundStyle(.red)
                Button("Retry") {
                    Task { await modelStore.retryDownload() }
                }
            }
        case .ready:
            EmptyView()
        }
    }

    private var pickerSection: some View {
        Section {
            PhotosPicker(selection: $pickerItem, matching: .any(of: [.images, .videos])) {
                Label(selection.selectedPhoto == nil && selection.selectedVideo == nil ? "Choose Photo or Video" : "Change Selection",
                      systemImage: "photo.on.rectangle")
            }
            .disabled(isRunning)
            if let selectedPhoto = selection.selectedPhoto {
                Image(uiImage: selectedPhoto)
                    .resizable()
                    .scaledToFit()
                    .frame(maxHeight: 220)
            } else if selection.selectedVideo != nil {
                Label("Video selected", systemImage: "video.fill")
            }
            if selection.isLoading {
                HStack {
                    ProgressView()
                    Text("Loading selection…")
                }
                .font(.caption)
            }
            if let errorMessage = selection.errorMessage {
                Text(errorMessage).foregroundStyle(.red).font(.caption)
            }
        }
    }

    private var samplingSection: some View {
        Section("Sampling") {
            if selection.selectedVideo != nil {
                Toggle("Single frame only", isOn: $isSingleFrameMode)
                if !isSingleFrameMode {
                    Toggle("Analyse movement (every frame, up to \(Int(FrameSource.analysisWindowSeconds)) s)",
                           isOn: $useNativeWindow)
                    if useNativeWindow {
                        // The trade this toggle makes, including the high-rate
                        // cap and cost. The sentence lives beside FrameSource's
                        // arithmetic and is pinned by OfflineDisclosureTests.
                        Text(FrameSource.nativeWindowDisclosure)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    } else {
                        Stepper(value: $fps, in: 0.5...10.0, step: 0.5) {
                            Text(String(format: "%.1f frames / second", fps))
                        }
                        Text("Sparse sampling over the whole clip — for a held pose, not for movement. Processing time rises with the selected frame count; actual iPhone run time has not been measured.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
                if let notice = runner.frameBudgetNotice {
                    // Names which of the two causes fired and how many frames
                    // were really used. The single sentence this replaced said
                    // "longer than the analysis window" and "120 frames" for
                    // every case, including the short-clip and exact-window
                    // cases where the cause is not clip length. See
                    // `FrameBudgetNotice`.
                    Text(notice.message)
                        .font(.caption)
                        .foregroundStyle(.orange)
                }
            } else {
                Text("A photo is always processed as a single frame.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }

    private var runSection: some View {
        Section {
            Button(action: startRun) {
                runButtonLabel
            }
            .disabled(
                isRunning
                    || selection.isLoading
                    || !modelStore.state.allowsModelLoadAttempt
            )

            if isRunning {
                Button("Cancel", role: .destructive) { runner.cancel() }
            }
        }
    }

    @ViewBuilder
    private var runButtonLabel: some View {
        switch runner.phase {
        case .checkingCameraReference:
            HStack { ProgressView(); Text("Checking camera reference…") }
        case .loadingModel:
            HStack { ProgressView(); Text("Loading pose model…") }
        case .waitingForModel:
            Text(
                modelStore.state.allowsModelLoadAttempt
                    ? "Run"
                    : "Waiting for pose model…"
            )
        case .decodingFrames:
            HStack { ProgressView(); Text("Reading video…") }
        case .running(let current, let total, let etaSeconds):
            HStack {
                ProgressView()
                if let etaSeconds {
                    Text("Frame \(current + 1)/\(total) — about \(max(Int(etaSeconds.rounded()), 0))s left")
                } else {
                    Text("Frame \(current + 1)/\(total) — estimating time…")
                }
            }
        default:
            Text("Run")
        }
    }

    private var isRunning: Bool {
        switch runner.phase {
        case .checkingCameraReference, .loadingModel, .decodingFrames, .running:
            return true
        default: return false
        }
    }

    // MARK: - Actions

    private func close() {
        // Fence the shared Nimble engine synchronously before the containing
        // sheet/view can disappear and a later runner acquires it.
        selectionLoadTask?.cancel()
        runner.cancel()
        onDismiss()
    }

    private func startRun() {
        guard modelStore.state.allowsModelLoadAttempt else { return }
        if let selectedPhoto = selection.selectedPhoto {
            runner.run(source: .photo(selectedPhoto), samplingMode: .singleFrame)
        } else if let selectedVideo = selection.selectedVideo {
            let mode: FrameSource.SamplingMode
            if isSingleFrameMode {
                mode = .singleFrame
            } else if useNativeWindow {
                mode = .nativeWindow(seconds: FrameSource.analysisWindowSeconds)
            } else {
                mode = .fps(fps)
            }
            runner.run(source: .video(selectedVideo), samplingMode: mode)
        }
    }
}

/// `Transferable` wrapper that copies a picked video to a private temp file
/// rather than loading it into memory as `Data` (the standard Apple-documented
/// pattern for `PhotosPicker` video selection).
private struct PickedMovie: Transferable {
    let owner: AppOwnedTemporaryVideo

    static var transferRepresentation: some TransferRepresentation {
        FileRepresentation(importedContentType: .movie) { received in
            // `received.file` is borrowed from the Photos transfer provider and
            // may disappear as soon as this closure returns. Finish the copy
            // synchronously here; only the app-owned lifetime token escapes.
            Self(owner: try AppOwnedTemporaryVideo(copying: received.file))
        }
    }
}
