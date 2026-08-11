import XCTest
import UIKit
@testable import BioMotion

final class OfflineImportLifecycleTests: XCTestCase {
    private func makeTemporaryRoot() throws -> URL {
        let root = FileManager.default.temporaryDirectory
            .appendingPathComponent(
                "biomotion-offline-import-tests-\(UUID().uuidString)",
                isDirectory: true
            )
        try FileManager.default.createDirectory(
            at: root,
            withIntermediateDirectories: false,
            attributes: [.posixPermissions: 0o700]
        )
        return root
    }

    private func makeSource(in root: URL, name: String = "provider.mov") throws -> URL {
        let url = root.appendingPathComponent(name, isDirectory: false)
        try Data("picked-video-bytes".utf8).write(to: url, options: .atomic)
        return url
    }

    func testOwnedCopyUsesUniquePrivateDirectoryAndPreservesBytes() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let source = try makeSource(in: root)
        let sourceBytes = try Data(contentsOf: source)

        let first = try AppOwnedTemporaryVideo(copying: source, into: root)
        let second = try AppOwnedTemporaryVideo(copying: source, into: root)

        XCTAssertNotEqual(first.url, second.url)
        XCTAssertNotEqual(first.url.deletingLastPathComponent(),
                          second.url.deletingLastPathComponent())
        XCTAssertEqual(try Data(contentsOf: first.url), sourceBytes)
        XCTAssertEqual(try Data(contentsOf: second.url), sourceBytes)

        for directory in [first.url.deletingLastPathComponent(),
                          second.url.deletingLastPathComponent()] {
            let attributes = try FileManager.default.attributesOfItem(atPath: directory.path)
            XCTAssertEqual((attributes[.posixPermissions] as? NSNumber)?.intValue,
                           0o700)
        }
    }

    func testOwnedDirectoryLivesUntilLastReferenceThenDeletesWithoutDeletingProvider() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let source = try makeSource(in: root)

        var first: AppOwnedTemporaryVideo? = try AppOwnedTemporaryVideo(
            copying: source,
            into: root
        )
        let directory = try XCTUnwrap(first?.url.deletingLastPathComponent())
        weak var weakOwner = first
        var lastReference = first

        first = nil
        XCTAssertNotNil(weakOwner)
        XCTAssertTrue(FileManager.default.fileExists(atPath: directory.path))

        lastReference = nil
        XCTAssertNil(weakOwner)
        XCTAssertFalse(FileManager.default.fileExists(atPath: directory.path))
        XCTAssertTrue(FileManager.default.fileExists(atPath: source.path),
                      "the Photos provider file is borrowed, never owned")
    }

    func testFailedCopyRemovesItsFreshDirectory() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let missing = root.appendingPathComponent("missing.mov")

        XCTAssertThrowsError(
            try AppOwnedTemporaryVideo(copying: missing, into: root)
        )
        XCTAssertEqual(try FileManager.default.contentsOfDirectory(
            at: root,
            includingPropertiesForKeys: nil
        ), [])
    }

    @MainActor
    func testLateSelectionCannotReplaceTheLatestSelection() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let source = try makeSource(in: root)
        let staleVideo = try AppOwnedTemporaryVideo(copying: source, into: root)
        let latestVideo = try AppOwnedTemporaryVideo(copying: source, into: root)
        var state = OfflineImportSelectionState()

        let staleGeneration = state.beginLoading()
        let latestGeneration = state.beginLoading()

        XCTAssertFalse(state.commit(video: staleVideo, generation: staleGeneration))
        XCTAssertNil(state.selectedVideo)
        XCTAssertTrue(state.isLoading)

        XCTAssertTrue(state.commit(video: latestVideo, generation: latestGeneration))
        XCTAssertTrue(state.selectedVideo === latestVideo)
        XCTAssertFalse(state.isLoading)
    }

    @MainActor
    func testStaleFailureCannotClearLatestLoadingState() {
        var state = OfflineImportSelectionState()
        let staleGeneration = state.beginLoading()
        _ = state.beginLoading()

        XCTAssertFalse(state.fail("A failed", generation: staleGeneration))
        XCTAssertTrue(state.isLoading)
        XCTAssertNil(state.errorMessage)
    }

    @MainActor
    func testFailureAndCancellationKeepThePreviousSelectionForRetry() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let source = try makeSource(in: root)
        let previous = try AppOwnedTemporaryVideo(copying: source, into: root)
        var state = OfflineImportSelectionState()

        let first = state.beginLoading()
        XCTAssertTrue(state.commit(video: previous, generation: first))

        let failed = state.beginLoading()
        XCTAssertTrue(state.fail("B failed", generation: failed))
        XCTAssertTrue(state.selectedVideo === previous)
        XCTAssertEqual(state.errorMessage, "B failed")

        let cancelled = state.beginLoading()
        XCTAssertTrue(state.cancel(generation: cancelled))
        XCTAssertTrue(state.selectedVideo === previous)
        XCTAssertFalse(state.isLoading)
        XCTAssertNil(state.errorMessage)
        XCTAssertFalse(state.commit(video: previous, generation: cancelled),
                       "a cancelled load may not publish late")
    }

    @MainActor
    func testSuccessfulReplacementReleasesThePreviousOwner() throws {
        let root = try makeTemporaryRoot()
        defer { try? FileManager.default.removeItem(at: root) }
        let source = try makeSource(in: root)
        var previous: AppOwnedTemporaryVideo? = try AppOwnedTemporaryVideo(
            copying: source,
            into: root
        )
        weak var weakPrevious = previous
        let previousDirectory = try XCTUnwrap(
            previous?.url.deletingLastPathComponent()
        )
        let replacement = try AppOwnedTemporaryVideo(copying: source, into: root)
        var state = OfflineImportSelectionState()

        let first = state.beginLoading()
        XCTAssertTrue(state.commit(video: try XCTUnwrap(previous), generation: first))
        previous = nil
        XCTAssertNotNil(weakPrevious, "selection state must retain the current owner")

        let second = state.beginLoading()
        XCTAssertTrue(state.commit(video: replacement, generation: second))
        XCTAssertNil(weakPrevious)
        XCTAssertFalse(FileManager.default.fileExists(atPath: previousDirectory.path))
        XCTAssertTrue(state.selectedVideo === replacement)
    }
}
