import BackgroundAssets
import Foundation
import System

/// Resolves the SAM 3D Body Core ML model, which is **not** in the app bundle.
///
/// # Why
/// `SAM3DBodyPose.mlmodelc` is 1.31 GiB — it was 99.5% of a 1.3151 GiB app
/// download (measured on the 2026-08-07 Release archive). The weights never
/// change between app versions, so they now ship as an Apple-Hosted **Managed
/// Background Assets** pack (iOS 26): the OS fetches the pack from Apple's CDN
/// and stores it outside the app bundle, and app updates no longer re-download
/// it. ODR is not used — it is deprecated as of iOS 27.
///
/// # What is in the pack: a PRE-COMPILED model
/// Asset delivery has no Xcode build step. Both production packs and local
/// developer bundles therefore carry the already-compiled directory produced
/// and receipt-verified by `tools/assetpack/package.sh` rather than accepting a
/// source model at runtime, because:
///
///   * `.mlmodelc` is byte-for-byte the artifact Xcode already put in the app
///     bundle, loaded by exactly the same `MLModel(contentsOf:)` call that has
///     been running on this project's device builds. Nothing about the load
///     path changes — only where the directory lives.
///   * On-device compilation of a 1.3 GiB package would need the package
///     *and* its compiled output resident at once (~2.6 GiB of the user's
///     disk), and the output would have to live in the app container, where
///     the OS cannot evict or update it. Keeping only `.mlmodelc`, inside the
///     system-managed asset container, halves the disk cost and lets iOS
///     manage it.
///   * Compilation would land on the user's first import, adding a long,
///     unattributable stall to an action that already runs a heavy pipeline.
///
/// # Resolution order (first hit wins)
///   1. `SAM3DBodyPose.mlmodelc` in the app bundle  — developer builds and the
///      Simulator, where Background Assets serves no packs at all. Populate it
///      with `tools/assetpack/dev_bundle_model.sh on`.
///   2. `SAM3DBodyPose.mlmodelc` in the asset pack   — the shipping path.
///   3. Nothing available: kick the download off and throw `.downloading`
///      carrying live progress, or `.unavailable` with the real reason.
///
/// # This never blocks on a 1.31 GiB download
/// `resolveCompiledModelURL()` returns (or throws) promptly. When the pack is
/// missing it starts the transfer in the background and reports the percentage
/// so far, so the caller's UI shows a real, advancing number instead of an
/// indefinite spinner. Retrying picks up the download already in flight.
@MainActor
final class AssetPackModelStore: ObservableObject {

    static let shared = AssetPackModelStore()

    /// Must match `AssetPackIdentity.modelPackID` in the Downloader extension
    /// and `assetPackID` in `tools/assetpack/Manifest.json`.
    nonisolated static let assetPackID = "sam3d-body-pose"
    nonisolated static let modelBaseName = "SAM3DBodyPose"
    nonisolated static let compiledModelFileName = "SAM3DBodyPose.mlmodelc"
    nonisolated static let compiledModelInteriorFileName = "coremldata.bin"

    /// Where the model came from — surfaced so a bug report can say whether a
    /// device was running the bundled developer copy or the shipping pack.
    enum Provenance: String {
        case bundledCompiled = "app bundle (compiled)"
        case assetPackCompiled = "asset pack"
    }

    enum State: Equatable {
        case idle
        case ready(Provenance)
        case downloading(fraction: Double, receivedBytes: Int64, totalBytes: Int64)
        case unavailable(String)
    }

    /// Observable so a view can render live progress. Nothing in the app
    /// observes it yet; `OfflineImportView` currently surfaces the same
    /// information through the message on the thrown `Unavailable` error. A
    /// live progress bar is one `@ObservedObject var store = AssetPackModelStore.shared`
    /// away.
    @Published private(set) var state: State = .idle

    /// Thrown instead of blocking. `message` is written to be shown verbatim to
    /// the user.
    struct Unavailable: LocalizedError {
        let message: String
        var errorDescription: String? { message }
        /// True while a transfer is in flight — the caller can offer "try again"
        /// rather than treating this as a hard failure.
        let isDownloading: Bool
    }

    private var downloadTask: Task<Void, Never>?
    private var progressTask: Task<Void, Never>?

    private init() {}

    // MARK: - Entry point

    /// Returns a URL to a **compiled** Core ML model directory, or throws
    /// `Unavailable` with a user-facing message. Idempotent and cheap after the
    /// first success.
    func resolveCompiledModelURL() async throws -> URL {
        if let url = Self.bundledCompiledModelURL() {
            state = .ready(.bundledCompiled)
            return url
        }
        if let url = await Self.probeAssetPack() {
            cancelProgressWatch()
            state = .ready(.assetPackCompiled)
            return url
        }
        throw startDownloadAndDescribe()
    }

    /// Same work as `resolveCompiledModelURL()` but the result is discarded —
    /// call it as soon as the model is *plausibly* about to be needed so the
    /// download has a head start. Logs the outcome; that log line is the only
    /// way to tell a Simulator/dev build apart from a real pack load without a
    /// debugger attached.
    func beginPrefetch() async {
        do {
            _ = try await resolveCompiledModelURL()
            NSLog("[AssetPack] pose model ready via %@", String(describing: state))
        } catch {
            NSLog("[AssetPack] pose model not ready: %@", error.localizedDescription)
        }
    }

    /// Off the main actor: `AssetPackManager.url(for:)` is synchronous and hits
    /// the filesystem / asset index, so probing it inline would block the UI.
    /// (The sibling locate-anything-ios store hit exactly this.)
    private nonisolated static func probeAssetPack() async -> URL? {
        await Task.detached(priority: .userInitiated) { () -> URL? in
            assetPackURL(for: compiledModelFileName)
        }.value
    }

    // MARK: - App bundle (developer / Simulator path)

    private nonisolated static func bundledCompiledModelURL() -> URL? {
        guard let url = Bundle.main.url(forResource: modelBaseName, withExtension: "mlmodelc") else { return nil }
        return FileManager.default.fileExists(atPath: url.path) ? url : nil
    }

    // MARK: - Asset pack lookup

    /// `AssetPackManager.url(for:)` is documented against files. `.mlmodelc` is a
    /// *directory*, and `ba-package`'s directory selector does preserve it as a
    /// directory entry (verified locally: the archive lists
    /// `Contents/SAM3DBodyPose.mlmodelc` and its children). Asking for the
    /// directory path directly is therefore the expected call — but it is the
    /// one thing here that cannot be exercised without a device, because
    /// Background Assets serves no packs in the Simulator. If it turns out the
    /// manager only indexes leaf files, `coremldata.bin` recovers the directory
    /// from a file that is guaranteed to exist inside every compiled model.
    private nonisolated static func assetPackURL(for fileName: String) -> URL? {
        if let direct = try? AssetPackManager.shared.url(for: FilePath(fileName)),
           FileManager.default.fileExists(atPath: direct.path) {
            return direct
        }
        guard let inner = try? AssetPackManager.shared.url(
            for: FilePath("\(fileName)/\(compiledModelInteriorFileName)")
        ) else { return nil }
        let container = inner.deletingLastPathComponent()
        return FileManager.default.fileExists(atPath: container.path) ? container : nil
    }

    // MARK: - Download

    /// Starts (or joins) the pack download and returns the error to throw right
    /// now. Never waits for the transfer.
    private func startDownloadAndDescribe() -> Unavailable {
        if downloadTask == nil {
            startProgressWatch()
            downloadTask = Task { [weak self] in
                do {
                    let pack = try await AssetPackManager.shared.assetPack(withID: Self.assetPackID)
                    try await AssetPackManager.shared.ensureLocalAvailability(of: pack)
                    // The OS now says the pack is local. If the model still is
                    // not in it, the uploaded pack and this build disagree —
                    // a permanent failure that must NOT keep reporting
                    // "downloading…", which is what the user would otherwise
                    // see forever.
                    let compiledModel = await Self.probeAssetPack()
                    await MainActor.run {
                        self?.downloadTask = nil
                        if compiledModel == nil {
                            self?.cancelProgressWatch()
                            self?.state = .unavailable(
                                "The pose model pack downloaded but contains no \(Self.compiledModelFileName). "
                                + "The uploaded asset pack and this app build disagree — repackage with "
                                + "tools/assetpack/package.sh and re-upload.")
                        }
                    }
                } catch {
                    await MainActor.run {
                        self?.downloadTask = nil
                        self?.cancelProgressWatch()
                        self?.state = .unavailable(Self.describe(error))
                    }
                }
            }
        }

        switch state {
        case .downloading(let fraction, let received, let total):
            return Unavailable(message: Self.progressMessage(fraction: fraction, received: received, total: total),
                               isDownloading: true)
        case .unavailable(let reason):
            return Unavailable(message: reason, isDownloading: false)
        default:
            // The request was only just issued — whether it becomes a download
            // or an immediate failure is not known yet, and claiming
            // "downloading" here would be a guess. (Observed in the Simulator:
            // the lookup fails ~10 ms later with "No team ID was specified for
            // the app", which the next attempt then reports accurately.)
            return Unavailable(
                message: "The 1.3 GB pose model isn't on this device yet — the download was just "
                    + "requested. Try again in a moment for progress, or the reason it can't start.",
                isDownloading: true)
        }
    }

    private func startProgressWatch() {
        guard progressTask == nil else { return }
        progressTask = Task { [weak self] in
            for await update in AssetPackManager.shared.statusUpdates(forAssetPackWithID: Self.assetPackID) {
                guard let self else { return }
                await MainActor.run { self.apply(update) }
            }
        }
    }

    private func cancelProgressWatch() {
        progressTask?.cancel()
        progressTask = nil
    }

    private func apply(_ update: AssetPackManager.DownloadStatusUpdate) {
        switch update {
        case .downloading(_, let progress):
            state = .downloading(fraction: progress.fractionCompleted,
                                 receivedBytes: progress.completedUnitCount,
                                 totalBytes: progress.totalUnitCount)
        case .failed(_, let error):
            state = .unavailable(Self.describe(error))
        case .began, .paused, .finished:
            break
        @unknown default:
            break
        }
    }

    private nonisolated static func progressMessage(fraction: Double, received: Int64, total: Int64) -> String {
        let percent = Int((fraction * 100).rounded())
        if total > 0 {
            let mb = { (b: Int64) in Int(Double(b) / 1_048_576.0) }
            return "Downloading the pose model — \(percent)% (\(mb(received)) MB of \(mb(total)) MB). "
                + "It keeps going in the background; try again when it finishes."
        }
        return "Downloading the pose model — \(percent)%. "
            + "It keeps going in the background; try again when it finishes."
    }

    /// Turns a Background Assets error into something a user can act on. The
    /// `assetPackNotFound` case is by far the most common and the most
    /// misleading if left raw: it is what you get on the Simulator, on a build
    /// whose asset pack has not finished processing in App Store Connect, and on
    /// a sideloaded/dev build that never went through the store.
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
        // Everything else arrives as a raw NSError whose text is written for a
        // developer, not a user — e.g. the Simulator/locally-signed case is
        // "No team ID was specified for the app with the bundle ID …"
        // (observed 2026-08-07). Frame it rather than showing it bare.
        return "Couldn't get the pose model from the App Store: \(error.localizedDescription)"
    }

}
