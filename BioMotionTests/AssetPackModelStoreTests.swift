import XCTest
@testable import BioMotion

final class AssetPackModelStoreTests: XCTestCase {
    private enum DriverError: Error {
        case unavailable
    }

    private final class Driver: @unchecked Sendable {
        private let lock = NSLock()
        private var modelURL: URL?
        private var statusContinuations: [
            AsyncStream<AssetPackModelStore.DownloadEvent>.Continuation
        ] = []
        private var ensureContinuations: [CheckedContinuation<Void, Error>] = []
        private var controlsProbes = false
        private var probeContinuations: [CheckedContinuation<URL?, Never>] = []

        func setModelURL(_ url: URL?) {
            lock.lock()
            modelURL = url
            lock.unlock()
        }

        func probeModelURL() async -> URL? {
            let (isControlled, currentURL) = probeConfiguration()
            guard isControlled else { return currentURL }

            return await withCheckedContinuation { continuation in
                lock.lock()
                probeContinuations.append(continuation)
                lock.unlock()
            }
        }

        private func probeConfiguration() -> (Bool, URL?) {
            lock.lock()
            let isControlled = controlsProbes
            let currentURL = modelURL
            lock.unlock()
            return (isControlled, currentURL)
        }

        func enableControlledProbes() {
            lock.lock()
            controlsProbes = true
            lock.unlock()
        }

        var probeCallCount: Int {
            lock.lock()
            defer { lock.unlock() }
            return probeContinuations.count
        }

        func finishProbe(_ attempt: Int, with url: URL?) {
            lock.lock()
            let continuation = probeContinuations[attempt]
            lock.unlock()
            continuation.resume(returning: url)
        }

        func makeStatusStream() -> AsyncStream<AssetPackModelStore.DownloadEvent> {
            AsyncStream { continuation in
                lock.lock()
                statusContinuations.append(continuation)
                lock.unlock()
            }
        }

        func ensureLocalAvailability() async throws {
            try await withCheckedThrowingContinuation { continuation in
                lock.lock()
                ensureContinuations.append(continuation)
                lock.unlock()
            }
        }

        var statusStreamCount: Int {
            lock.lock()
            defer { lock.unlock() }
            return statusContinuations.count
        }

        var ensureCallCount: Int {
            lock.lock()
            defer { lock.unlock() }
            return ensureContinuations.count
        }

        func yield(
            _ event: AssetPackModelStore.DownloadEvent,
            attempt: Int
        ) {
            lock.lock()
            let continuation = statusContinuations[attempt]
            lock.unlock()
            continuation.yield(event)
        }

        func finishEnsure(_ attempt: Int, result: Result<Void, Error>) {
            lock.lock()
            let continuation = ensureContinuations[attempt]
            lock.unlock()
            continuation.resume(with: result)
        }
    }

    @MainActor
    private func makeStore(driver: Driver) -> AssetPackModelStore {
        AssetPackModelStore(
            dependencies: .init(
                bundledCompiledModelURL: { nil },
                assetPackCompiledModelURL: { await driver.probeModelURL() },
                ensureLocalAvailability: { try await driver.ensureLocalAvailability() },
                statusUpdates: { driver.makeStatusStream() }
            )
        )
    }

    @MainActor
    private func startMissingDownload(
        _ store: AssetPackModelStore,
        file: StaticString = #filePath,
        line: UInt = #line
    ) async {
        do {
            _ = try await store.resolveCompiledModelURL()
            XCTFail("a missing pack must not block until download completion", file: file, line: line)
        } catch let unavailable as AssetPackModelStore.Unavailable {
            XCTAssertTrue(unavailable.isDownloading, file: file, line: line)
        } catch {
            XCTFail("wrong missing-pack error: \(error)", file: file, line: line)
        }
    }

    @MainActor
    private func eventually(
        _ description: String,
        file: StaticString = #filePath,
        line: UInt = #line,
        _ predicate: @MainActor () -> Bool
    ) async {
        for _ in 0..<200 {
            if predicate() { return }
            await Task.yield()
            try? await Task.sleep(nanoseconds: 1_000_000)
        }
        XCTFail("timed out waiting for \(description)", file: file, line: line)
    }

    func testProgressFractionsAreFiniteAndSafeForProgressView() {
        XCTAssertEqual(AssetPackModelStore.DownloadProgress(fraction: .nan).fraction, 0)
        XCTAssertEqual(AssetPackModelStore.DownloadProgress(fraction: .infinity).fraction, 0)
        XCTAssertEqual(AssetPackModelStore.DownloadProgress(fraction: -0.2).fraction, 0)
        XCTAssertEqual(AssetPackModelStore.DownloadProgress(fraction: 1.2).fraction, 1)
    }

    @MainActor
    func testProgressUsesOnlyTheSystemFractionAndNeverAdvancesWithoutAnEvent() async {
        let driver = Driver()
        let store = makeStore(driver: driver)
        await startMissingDownload(store)
        XCTAssertEqual(
            driver.statusStreamCount,
            1,
            "the status observer must be installed before ensure can start"
        )
        await eventually("first download attempt") {
            driver.statusStreamCount == 1 && driver.ensureCallCount == 1
        }

        driver.yield(.downloading(fraction: 0.10), attempt: 0)
        await eventually("system progress") {
            if case .downloading(let progress) = store.state {
                return progress.fraction == 0.1
            }
            return false
        }

        let snapshot = store.state
        for _ in 0..<20 { await Task.yield() }
        XCTAssertEqual(store.state, snapshot, "there is no timer-based fake progress")
        XCTAssertTrue(store.state.message?.contains("10%") == true)
        XCTAssertFalse(store.state.message?.contains("MB") == true,
                       "Foundation.Progress unit counts are not documented as bytes")

        driver.finishEnsure(0, result: .failure(DriverError.unavailable))
        await eventually("failed attempt cleanup") {
            if case .unavailable = store.state { return true }
            return false
        }
    }

    @MainActor
    func testPausedStateRetainsLastRealProgressAndStopsClaimingItIsDownloading() async {
        let driver = Driver()
        let store = makeStore(driver: driver)
        await startMissingDownload(store)
        await eventually("first download attempt") {
            driver.statusStreamCount == 1 && driver.ensureCallCount == 1
        }

        driver.yield(.downloading(fraction: 0.25), attempt: 0)
        await eventually("progress before pause") {
            if case .downloading = store.state { return true }
            return false
        }
        driver.yield(.paused, attempt: 0)
        await eventually("paused state") {
            if case .paused(let progress) = store.state {
                return progress?.fraction == 0.25
            }
            return false
        }

        XCTAssertTrue(store.state.message?.contains("paused by iOS") == true)
        XCTAssertFalse(store.state.message?.contains("keeps going") == true)
        XCTAssertFalse(store.state.allowsModelLoadAttempt)

        driver.finishEnsure(0, result: .failure(DriverError.unavailable))
        await eventually("paused attempt cleanup") {
            if case .unavailable = store.state { return true }
            return false
        }
    }

    @MainActor
    func testFailureRetryIsSingleFlightAndLateAttemptACallbacksCannotOverwriteB() async {
        let driver = Driver()
        let store = makeStore(driver: driver)
        await startMissingDownload(store)
        await eventually("attempt A") {
            driver.statusStreamCount == 1 && driver.ensureCallCount == 1
        }

        driver.yield(.failed("A failed"), attempt: 0)
        await eventually("attempt A failure") {
            store.state == .unavailable("A failed")
        }
        XCTAssertFalse(store.state.allowsModelLoadAttempt,
                       "a hard failure is recovered only through explicit Retry")

        await store.retryDownload()
        await store.retryDownload()
        await eventually("one retry attempt B") {
            driver.statusStreamCount == 2 && driver.ensureCallCount == 2
        }
        XCTAssertEqual(driver.statusStreamCount, 2)
        XCTAssertEqual(driver.ensureCallCount, 2)
        XCTAssertEqual(store.state, .checking)

        driver.yield(.downloading(fraction: 0.4), attempt: 1)
        await eventually("attempt B progress") {
            if case .downloading(let progress) = store.state {
                return progress.fraction == 0.4
            }
            return false
        }

        driver.yield(.failed("late A failure"), attempt: 0)
        driver.yield(.downloading(fraction: 0.01), attempt: 0)
        for _ in 0..<20 { await Task.yield() }
        guard case .downloading(let progress) = store.state else {
            return XCTFail("attempt B progress was overwritten: \(store.state)")
        }
        XCTAssertEqual(progress.fraction, 0.4)

        let modelURL = URL(fileURLWithPath: "/tmp/SAM3DBodyPose.mlmodelc")
        driver.setModelURL(modelURL)
        driver.finishEnsure(1, result: .success(()))
        await eventually("attempt B ready") {
            store.state == .ready(.assetPackCompiled)
        }

        driver.finishEnsure(0, result: .failure(DriverError.unavailable))
        for _ in 0..<20 { await Task.yield() }
        XCTAssertEqual(store.state, .ready(.assetPackCompiled))
    }

    @MainActor
    func testFinishedAttemptAutomaticallyProbesAndBecomesReadyWithoutSecondRun() async {
        let driver = Driver()
        let store = makeStore(driver: driver)
        await startMissingDownload(store)
        await eventually("download attempt") {
            driver.statusStreamCount == 1 && driver.ensureCallCount == 1
        }

        driver.yield(.downloading(fraction: 0.9), attempt: 0)
        driver.yield(.finished, attempt: 0)
        let modelURL = URL(fileURLWithPath: "/tmp/SAM3DBodyPose.mlmodelc")
        driver.setModelURL(modelURL)
        driver.finishEnsure(0, result: .success(()))

        await eventually("automatic ready state") {
            store.state == .ready(.assetPackCompiled)
        }
        driver.yield(.failed("late terminal event"), attempt: 0)
        for _ in 0..<20 { await Task.yield() }
        XCTAssertEqual(store.state, .ready(.assetPackCompiled))
        XCTAssertTrue(store.state.allowsModelLoadAttempt)
    }

    @MainActor
    func testFreshStoreReattachesToSystemManagedDownload() async {
        let driver = Driver()
        let store = makeStore(driver: driver)

        await startMissingDownload(store)
        XCTAssertEqual(store.state, .checking)
        await eventually("reattached observer and ensure call") {
            driver.statusStreamCount == 1 && driver.ensureCallCount == 1
        }
        driver.yield(.downloading(fraction: 0.7), attempt: 0)
        await eventually("continued system progress") {
            if case .downloading(let progress) = store.state {
                return progress.fraction == 0.7
            }
            return false
        }

        driver.finishEnsure(0, result: .failure(DriverError.unavailable))
        await eventually("reattached attempt cleanup") {
            if case .unavailable = store.state { return true }
            return false
        }
    }

    @MainActor
    func testAlreadyLocalPackNeedsNoDownloadAttempt() async throws {
        let driver = Driver()
        let modelURL = URL(fileURLWithPath: "/tmp/SAM3DBodyPose.mlmodelc")
        driver.setModelURL(modelURL)
        let store = makeStore(driver: driver)

        let resolved = try await store.resolveCompiledModelURL()

        XCTAssertEqual(resolved, modelURL)
        XCTAssertEqual(store.state, .ready(.assetPackCompiled))
        XCTAssertEqual(driver.statusStreamCount, 0)
        XCTAssertEqual(driver.ensureCallCount, 0)
    }

    @MainActor
    func testReentrantStaleProbeCannotReplaceAReadyModelOrStartDownload() async {
        let driver = Driver()
        driver.enableControlledProbes()
        let store = makeStore(driver: driver)
        let modelURL = URL(fileURLWithPath: "/tmp/SAM3DBodyPose.mlmodelc")

        let first = Task { try await store.resolveCompiledModelURL() }
        await eventually("first suspended model probe") {
            driver.probeCallCount == 1
        }
        let second = Task { try await store.resolveCompiledModelURL() }
        await eventually("second suspended model probe") {
            driver.probeCallCount == 2
        }

        driver.finishProbe(1, with: modelURL)
        do {
            let resolved = try await second.value
            XCTAssertEqual(resolved, modelURL)
        } catch {
            XCTFail("the winning probe should resolve the model: \(error)")
        }
        XCTAssertEqual(store.state, .ready(.assetPackCompiled))

        driver.finishProbe(0, with: nil)
        do {
            let resolved = try await first.value
            XCTAssertEqual(resolved, modelURL)
        } catch {
            XCTFail("the stale probe must reuse the model published while it awaited: \(error)")
        }
        XCTAssertEqual(store.state, .ready(.assetPackCompiled))
        XCTAssertEqual(driver.statusStreamCount, 0)
        XCTAssertEqual(driver.ensureCallCount, 0)
    }

    func testOfflineImportObservesLiveStateAndDoesNotTreatWaitingAsModelFailure() throws {
        let root = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let view = try String(
            contentsOf: root.appendingPathComponent("BioMotion/Offline/OfflineImportView.swift"),
            encoding: .utf8
        )
        let runner = try String(
            contentsOf: root.appendingPathComponent("BioMotion/Offline/OfflineSessionRunner.swift"),
            encoding: .utf8
        )

        XCTAssertTrue(view.contains("@ObservedObject private var modelStore"))
        XCTAssertTrue(view.contains("ProgressView(value: progress.fraction)"))
        XCTAssertTrue(view.contains("await modelStore.retryDownload()"))
        let runGate = try XCTUnwrap(
            view.range(of: ".disabled(\n                isRunning")
        )
        let startRun = try XCTUnwrap(
            view.range(of: "private func startRun()")
        )
        XCTAssertTrue(
            view[runGate.lowerBound..<startRun.lowerBound]
                .contains("!modelStore.state.allowsModelLoadAttempt"),
            "the visible Run control, not an unrelated token, must own the model gate"
        )
        let startRunBody = view[startRun.lowerBound...]
        XCTAssertTrue(
            startRunBody.hasPrefix(
                "private func startRun() {\n"
                    + "        guard modelStore.state.allowsModelLoadAttempt else { return }"
            ),
            "the action must repeat the Run gate for programmatic calls"
        )
        XCTAssertFalse(view.contains("modelStore.cancel"),
                       "closing the import sheet must not cancel the OS-managed download")
        let load = try XCTUnwrap(
            runner.range(of: "try await poseEstimator.loadModelIfNeeded()")
        )
        let unavailableCatch = try XCTUnwrap(
            runner.range(
                of: "catch is AssetPackModelStore.Unavailable",
                range: load.lowerBound..<runner.endIndex
            )
        )
        let genericCatch = try XCTUnwrap(
            runner.range(
                of: "} catch {",
                range: unavailableCatch.upperBound..<runner.endIndex
            )
        )
        let availabilityBranch = runner[
            unavailableCatch.lowerBound..<genericCatch.lowerBound
        ]
        XCTAssertTrue(availabilityBranch.contains("setPhase(.waitingForModel"))
        XCTAssertTrue(
            availabilityBranch.trimmingCharacters(in: .whitespacesAndNewlines)
                .hasSuffix("return"),
            "normal download wait must return before the model-failure catch"
        )
    }
}
