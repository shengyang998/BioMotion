import BackgroundAssets
import Combine
import Foundation
import System

/// Resolves the SAM 3D Body Core ML model, which is **not** in the app bundle.
///
/// Shipping builds obtain the precompiled model from an Apple-hosted Managed
/// Background Assets pack. `resolveCompiledModelURL()` never waits for the
/// 1.3 GiB transfer: it either returns a model that is already local or starts
/// (or joins) one system-managed attempt and throws `Unavailable` immediately.
/// Its progress fraction is driven only by Background Assets events; there is
/// no timer-based or byte-count-derived synthetic progress.
@MainActor
final class AssetPackModelStore: ObservableObject {

    nonisolated static let assetPackID = "sam3d-body-pose"
    nonisolated static let modelBaseName = "SAM3DBodyPose"
    nonisolated static let compiledModelFileName = "SAM3DBodyPose.mlmodelc"
    nonisolated static let compiledModelInteriorFileName = "coremldata.bin"

    enum Provenance: String, Equatable, Sendable {
        case bundledCompiled = "app bundle (compiled)"
        case assetPackCompiled = "asset pack"
    }

    /// Progress reported by Background Assets. Foundation's `Progress` unit
    /// counts are deliberately not exposed: Apple does not document them as
    /// bytes, so presenting them as MB would invent information.
    struct DownloadProgress: Equatable, Sendable {
        let fraction: Double

        init(fraction: Double) {
            guard fraction.isFinite else {
                self.fraction = 0
                return
            }
            self.fraction = min(max(fraction, 0), 1)
        }
    }

    enum State: Equatable, Sendable {
        /// Looking up the pack, waiting for the first status event, or verifying
        /// a `finished` event by probing the model leaf.
        case checking
        case downloading(DownloadProgress)
        /// The last real system fraction is retained when one exists, but the
        /// message does not claim that a paused transfer is still advancing.
        case paused(DownloadProgress?)
        case ready(Provenance)
        case unavailable(String)

        var message: String? {
            switch self {
            case .checking:
                return "Checking the pose model download…"
            case .downloading(let progress):
                return AssetPackModelStore.progressMessage(progress)
            case .paused(let progress):
                let suffix = progress.map {
                    " at \(AssetPackModelStore.percent($0.fraction))%"
                } ?? ""
                return "The pose model download was paused by iOS\(suffix). "
                    + "It will resume when system conditions allow."
            case .ready:
                return nil
            case .unavailable(let reason):
                return reason
            }
        }

        /// Loading unless the model is ready would only repeat `Unavailable`.
        /// A terminal diagnostic has its own explicit `retryDownload()` path.
        var allowsModelLoadAttempt: Bool {
            switch self {
            case .ready:
                return true
            case .checking, .downloading, .paused, .unavailable:
                return false
            }
        }
    }

    /// Small, Sendable surface between the live Background Assets adapter and
    /// the deterministic state machine used by tests.
    enum DownloadEvent: Equatable, Sendable {
        case began
        case paused
        case downloading(fraction: Double)
        case finished
        case failed(String)
    }

    struct Dependencies: Sendable {
        let bundledCompiledModelURL: @Sendable () -> URL?
        let assetPackCompiledModelURL: @Sendable () async -> URL?
        let ensureLocalAvailability: @Sendable () async throws -> Void
        let statusUpdates: @Sendable () -> AsyncStream<DownloadEvent>

        nonisolated static var live: Dependencies {
            Dependencies(
                bundledCompiledModelURL: {
                    AssetPackModelStore.bundledCompiledModelURL()
                },
                assetPackCompiledModelURL: {
                    await AssetPackModelStore.probeAssetPack()
                },
                ensureLocalAvailability: {
                    try await AssetPackModelStore.ensureLiveAvailability()
                },
                statusUpdates: {
                    AssetPackModelStore.liveStatusUpdates()
                }
            )
        }
    }

    struct Unavailable: LocalizedError {
        let message: String
        let isDownloading: Bool
        var errorDescription: String? { message }
    }

    static let shared = AssetPackModelStore(dependencies: .live)

    @Published private(set) var state: State = .checking

    private let dependencies: Dependencies
    private var resolvedModelURL: URL?
    private var resolvedProvenance: Provenance?
    private var nextGeneration: UInt64 = 0
    private var activeGeneration: UInt64?
    private var lastProgress: DownloadProgress?
    private var ensureTask: Task<Void, Never>?
    private var progressTask: Task<Void, Never>?

    init(dependencies: Dependencies) {
        self.dependencies = dependencies
    }

    // MARK: - Entry points

    /// Returns a URL to a precompiled Core ML model that is already local. A
    /// missing model starts exactly one background attempt and throws promptly.
    func resolveCompiledModelURL() async throws -> URL {
        if let bundled = dependencies.bundledCompiledModelURL() {
            finishWithoutAttempt(url: bundled, provenance: .bundledCompiled)
            return bundled
        }
        if let resolvedModelURL {
            return resolvedModelURL
        }
        let packed = await dependencies.assetPackCompiledModelURL()
        // The actor is re-entrant across the probe. Another resolve/ensure may
        // have published the authoritative URL while this older lookup was
        // suspended; never let the stale continuation reopen an attempt or
        // replace that ready state.
        if let resolvedModelURL {
            return resolvedModelURL
        }
        if let packed {
            finishWithoutAttempt(url: packed, provenance: .assetPackCompiled)
            return packed
        }

        if activeGeneration == nil {
            // A terminal failure is stable until the user explicitly retries;
            // repeated model-load calls must not manufacture concurrent work.
            if case .unavailable(let reason) = state {
                throw Unavailable(message: reason, isDownloading: false)
            }
            startAttempt()
        }
        throw unavailableForCurrentState()
    }

    /// Explicit retry after a terminal failure. Repeated taps are single-flight.
    func retryDownload() async {
        guard activeGeneration == nil else { return }

        if let bundled = dependencies.bundledCompiledModelURL() {
            finishWithoutAttempt(url: bundled, provenance: .bundledCompiled)
            return
        }
        if resolvedModelURL != nil {
            state = .ready(resolvedProvenance ?? .assetPackCompiled)
            return
        }
        // `startAttempt` reserves the generation before synchronously moving
        // off the old diagnostic to `.checking`. The system ensure call rejoins
        // an existing transfer and returns promptly if it is already local, so
        // a separate async pre-probe would only widen the double-tap race.
        startAttempt()
    }

    func beginPrefetch() async {
        do {
            _ = try await resolveCompiledModelURL()
            NSLog("[AssetPack] pose model ready via %@", String(describing: state))
        } catch {
            NSLog("[AssetPack] pose model not ready: %@", error.localizedDescription)
        }
    }

    // MARK: - Attempt state machine

    private func startAttempt() {
        guard activeGeneration == nil else { return }

        nextGeneration &+= 1
        let generation = nextGeneration
        activeGeneration = generation
        lastProgress = nil
        state = .checking

        let dependencies = dependencies
        // Construct the per-pack sequence before starting `ensure`. The live
        // Background Assets sequence subscribes at construction time, so a
        // fast began/paused transition cannot happen in the gap between the
        // request and observer installation.
        let statusUpdates = dependencies.statusUpdates()
        progressTask = Task { [weak self, statusUpdates] in
            for await event in statusUpdates {
                guard !Task.isCancelled, let self else { return }
                self.apply(event, generation: generation)
                guard self.activeGeneration == generation else { return }
            }
        }

        ensureTask = Task { [weak self, dependencies] in
            do {
                try await dependencies.ensureLocalAvailability()
                let compiledModel = await dependencies.assetPackCompiledModelURL()
                guard let self else { return }
                if let compiledModel {
                    self.finishAttempt(
                        generation: generation,
                        state: .ready(.assetPackCompiled),
                        resolvedURL: compiledModel
                    )
                } else {
                    self.finishAttempt(
                        generation: generation,
                        state: .unavailable(Self.packMismatchMessage),
                        resolvedURL: nil
                    )
                }
            } catch {
                guard let self else { return }
                self.finishAttempt(
                    generation: generation,
                    state: .unavailable(Self.describe(error)),
                    resolvedURL: nil
                )
            }
        }
    }

    private func apply(_ event: DownloadEvent, generation: UInt64) {
        guard activeGeneration == generation else { return }

        switch event {
        case .began:
            state = .checking
        case .downloading(let fraction):
            let progress = DownloadProgress(fraction: fraction)
            lastProgress = progress
            state = .downloading(progress)
        case .paused:
            state = .paused(lastProgress)
        case .finished:
            // `finished` describes the transfer, not the contents. The matching
            // ensure completion performs the authoritative leaf probe.
            state = .checking
        case .failed(let reason):
            finishAttempt(
                generation: generation,
                state: .unavailable(reason),
                resolvedURL: nil
            )
        }
    }

    /// The generation guard is the authority boundary: a cancelled attempt's
    /// stream or checked continuation may complete later, but cannot touch B.
    private func finishAttempt(
        generation: UInt64,
        state terminalState: State,
        resolvedURL: URL?
    ) {
        guard activeGeneration == generation else { return }
        activeGeneration = nil
        lastProgress = nil
        if let resolvedURL {
            self.resolvedModelURL = resolvedURL
            resolvedProvenance = .assetPackCompiled
        }
        progressTask?.cancel()
        progressTask = nil
        ensureTask?.cancel()
        ensureTask = nil
        state = terminalState
    }

    private func finishWithoutAttempt(url: URL, provenance: Provenance) {
        activeGeneration = nil
        lastProgress = nil
        resolvedModelURL = url
        resolvedProvenance = provenance
        progressTask?.cancel()
        progressTask = nil
        ensureTask?.cancel()
        ensureTask = nil
        state = .ready(provenance)
    }

    private func unavailableForCurrentState() -> Unavailable {
        switch state {
        case .checking:
            return Unavailable(
                message: state.message ?? "Checking the pose model download…",
                isDownloading: true
            )
        case .downloading:
            return Unavailable(message: state.message ?? "Downloading the pose model…", isDownloading: true)
        case .paused:
            return Unavailable(message: state.message ?? "The pose model download is paused.", isDownloading: false)
        case .unavailable(let reason):
            return Unavailable(message: reason, isDownloading: false)
        case .ready:
            return Unavailable(
                message: "The pose model became ready; try the load again.",
                isDownloading: false
            )
        }
    }

    // MARK: - Live Background Assets adapter

    /// Looks up a leaf known to exist in every compiled model, then recovers its
    /// `.mlmodelc` parent directory. Background Assets also supports directory
    /// lookups, but a pack can merge content from shared directories; resolving
    /// the required leaf proves this particular model payload is present.
    private nonisolated static func assetPackURL() -> URL? {
        let leafPath = "\(compiledModelFileName)/\(compiledModelInteriorFileName)"
        guard let leaf = try? AssetPackManager.shared.url(for: FilePath(leafPath)),
              FileManager.default.fileExists(atPath: leaf.path) else {
            return nil
        }

        let container = leaf.deletingLastPathComponent()
        var isDirectory: ObjCBool = false
        guard FileManager.default.fileExists(
            atPath: container.path,
            isDirectory: &isDirectory
        ), isDirectory.boolValue else {
            return nil
        }
        return container
    }

    private nonisolated static func bundledCompiledModelURL() -> URL? {
        guard let url = Bundle.main.url(
            forResource: modelBaseName,
            withExtension: "mlmodelc"
        ) else {
            return nil
        }
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    private nonisolated static func probeAssetPack() async -> URL? {
        await Task.detached(priority: .userInitiated) {
            assetPackURL()
        }.value
    }

    private nonisolated static func ensureLiveAvailability() async throws {
        let manager = AssetPackManager.shared
        let pack = try await manager.assetPack(withID: assetPackID)
        if #available(iOS 26.4, *) {
            try await manager.ensureLocalAvailability(
                of: pack,
                requireLatestVersion: false
            )
        } else {
            // The labelled overload was introduced in 26.4. Shipping still
            // supports iOS 26.0 through this deprecated-but-required fallback.
            try await manager.ensureLocalAvailability(of: pack)
        }
    }

    private nonisolated static func liveStatusUpdates() -> AsyncStream<DownloadEvent> {
        let updates = AssetPackManager.shared.statusUpdates(
            forAssetPackWithID: assetPackID
        )
        return AsyncStream { continuation in
            let task = Task {
                for await update in updates {
                    if let event = liveEvent(update) {
                        continuation.yield(event)
                    }
                }
                continuation.finish()
            }
            continuation.onTermination = { @Sendable _ in
                task.cancel()
            }
        }
    }

    private nonisolated static func liveEvent(
        _ update: AssetPackManager.DownloadStatusUpdate
    ) -> DownloadEvent? {
        switch update {
        case .began:
            return .began
        case .paused:
            return .paused
        case .downloading(_, let progress):
            // `fractionCompleted` is the only documented normalized progress.
            // The accompanying unit counts are ignored, not presented as bytes.
            return .downloading(fraction: progress.fractionCompleted)
        case .finished:
            return .finished
        case .failed(_, let error):
            return .failed(describe(error))
        @unknown default:
            return nil
        }
    }

    // MARK: - Messages

    private nonisolated static func percent(_ fraction: Double) -> Int {
        Int((DownloadProgress(fraction: fraction).fraction * 100).rounded())
    }

    private nonisolated static func progressMessage(_ progress: DownloadProgress) -> String {
        "Downloading the pose model — \(percent(progress.fraction))%. "
            + "It continues in the background; Run becomes available when it is ready."
    }

    private nonisolated static let packMismatchMessage =
        "The pose model pack downloaded but contains no \(compiledModelFileName). "
        + "The uploaded asset pack and this app build disagree — repackage with "
        + "tools/assetpack/package.sh and re-upload."

    private nonisolated static func describe(_ error: Error) -> String {
        if let managed = error as? ManagedBackgroundAssetsError {
            switch managed {
            case .assetPackNotFound:
                return "The pose model isn't available to this build. Managed Background Assets only serves "
                    + "packs to App Store / TestFlight installs on a real device — the Simulator and locally "
                    + "signed builds get nothing. If this build did come from TestFlight, the asset pack may "
                    + "still be processing; try again in a few minutes."
            case .fileNotFound(let path):
                return "The pose model pack downloaded but doesn't contain \(path). "
                    + "The packaged asset pack and this app build disagree — repackage with "
                    + "tools/assetpack/package.sh."
            @unknown default:
                break
            }
        }
        return "Couldn't get the pose model from the App Store: \(error.localizedDescription)"
    }
}
