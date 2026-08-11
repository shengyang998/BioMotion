import Foundation
import UIKit

/// Owns one app-private copy of a video supplied by a picker provider.
///
/// The provider URL is borrowed. This owner removes only the private directory
/// it created, and keeps that directory alive until its final reference dies.
final class AppOwnedTemporaryVideo: Sendable {
    let url: URL

    private let directoryURL: URL

    init(
        copying sourceURL: URL,
        into rootDirectory: URL = FileManager.default.temporaryDirectory,
        identifier: UUID = UUID()
    ) throws {
        let fileManager = FileManager.default
        let directoryURL = rootDirectory.appendingPathComponent(
            "biomotion-import-\(identifier.uuidString)",
            isDirectory: true
        )

        try fileManager.createDirectory(
            at: directoryURL,
            withIntermediateDirectories: false,
            attributes: [.posixPermissions: 0o700]
        )

        do {
            let destinationURL = directoryURL.appendingPathComponent(
                sourceURL.lastPathComponent,
                isDirectory: false
            )
            try fileManager.copyItem(at: sourceURL, to: destinationURL)
            self.url = destinationURL
            self.directoryURL = directoryURL
        } catch {
            try? fileManager.removeItem(at: directoryURL)
            throw error
        }
    }

    deinit {
        try? FileManager.default.removeItem(at: directoryURL)
    }
}

/// Main-actor state machine for asynchronous picker loads.
///
/// A generation may publish only while it is the latest active load. Starting
/// a newer load, failing, or cancelling prevents late work from replacing the
/// retained selection.
@MainActor
struct OfflineImportSelectionState {
    private(set) var selectedPhoto: UIImage?
    private(set) var selectedVideo: AppOwnedTemporaryVideo?
    private(set) var errorMessage: String?
    private(set) var isLoading = false

    private var currentGeneration: UInt64 = 0

    init() {}

    mutating func beginLoading() -> UInt64 {
        currentGeneration &+= 1
        isLoading = true
        errorMessage = nil
        return currentGeneration
    }

    @discardableResult
    mutating func commit(photo: UIImage, generation: UInt64) -> Bool {
        guard accepts(generation) else { return false }

        selectedPhoto = photo
        selectedVideo = nil
        finishLoading()
        return true
    }

    @discardableResult
    mutating func commit(
        video: AppOwnedTemporaryVideo,
        generation: UInt64
    ) -> Bool {
        guard accepts(generation) else { return false }

        selectedVideo = video
        selectedPhoto = nil
        finishLoading()
        return true
    }

    @discardableResult
    mutating func fail(_ message: String, generation: UInt64) -> Bool {
        guard accepts(generation) else { return false }

        isLoading = false
        errorMessage = message
        return true
    }

    @discardableResult
    mutating func cancel(generation: UInt64) -> Bool {
        guard accepts(generation) else { return false }

        isLoading = false
        errorMessage = nil
        currentGeneration &+= 1
        return true
    }

    private func accepts(_ generation: UInt64) -> Bool {
        isLoading && generation == currentGeneration
    }

    private mutating func finishLoading() {
        isLoading = false
        errorMessage = nil
    }
}
