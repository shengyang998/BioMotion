import XCTest
@testable import BioMotion

final class MotionRecorderTests: XCTestCase {

    private func makeFrame(timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        let joints = [
            TrackedJoint(id: "hips_joint", name: "Pelvis", worldPosition: SIMD3(0, 1, 0), isTracked: true)
        ]
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    @MainActor
    private func loadedEngine() async -> NimbleEngine {
        let engine = NimbleEngine()
        engine.loadBundledModel()
        let deadline = Date().addingTimeInterval(120)
        while !engine.isModelLoaded && Date() < deadline {
            try? await Task.sleep(nanoseconds: 100_000_000)
        }
        XCTAssertTrue(engine.isModelLoaded, "model never finished loading")
        return engine
    }

    func testInitialState() {
        let recorder = MotionRecorder()
        XCTAssertFalse(recorder.isRecording)
        XCTAssertFalse(recorder.hasRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertEqual(recorder.duration, 0)
    }

    func testStartRecording() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        XCTAssertTrue(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
    }

    func testRecordFrames() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.recordFrame(makeFrame(timestamp: 0.033, frameNumber: 3))

        XCTAssertEqual(recorder.recordedFrameCount, 3)
        XCTAssertEqual(recorder.frames.count, 3)
        XCTAssertEqual(recorder.duration, 0.033, accuracy: 0.001)
    }

    func testIgnoresFramesWhenNotRecording() {
        let recorder = MotionRecorder()

        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))

        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertFalse(recorder.hasRecording)
    }

    func testStopRecording() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.stopRecording()

        XCTAssertFalse(recorder.isRecording)
        XCTAssertTrue(recorder.hasRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 2)
    }

    func testStopThenRecordIgnoresFrames() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.stopRecording()

        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        XCTAssertEqual(recorder.recordedFrameCount, 1)
    }

    /// Starting a replacement capture is destructive. The recorder must keep
    /// the completed take intact until the UI has obtained explicit discard
    /// confirmation instead of clearing it as a side effect of the red button.
    func testStartPreservesUnexportedRecordingUntilExplicitDiscard() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 0.016, frameNumber: 2))
        recorder.stopRecording()

        recorder.startRecording()
        XCTAssertFalse(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 2)
        XCTAssertTrue(recorder.hasRecording)
    }

    func testRecordingStopsAtFrameLimit() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        for index in 0...3_600 {
            recorder.recordFrame(
                makeFrame(timestamp: Double(index) / 60.0, frameNumber: index + 1)
            )
        }

        XCTAssertFalse(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 3_600)
        XCTAssertEqual(recorder.frames.count, 3_600)
    }

    func testRecordingStopsAtDurationLimit() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 100, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 160, frameNumber: 2))
        recorder.recordFrame(makeFrame(timestamp: 160.001, frameNumber: 3))

        XCTAssertFalse(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 2)
        XCTAssertEqual(recorder.duration, 60, accuracy: 0.000_001)
    }

    func testRealCaptureRejectsPreOriginAndWrongEpochFrames() {
        let recorder = MotionRecorder()
        let active = CaptureEpoch(timeOrigin: 100)
        let wrong = CaptureEpoch(timeOrigin: 100)
        XCTAssertTrue(recorder.startRecording(for: active))

        XCTAssertEqual(
            recorder.recordFrame(makeFrame(timestamp: 99.999, frameNumber: 1), for: active),
            .ignored
        )
        XCTAssertEqual(
            recorder.recordFrame(makeFrame(timestamp: 100.1, frameNumber: 2), for: wrong),
            .ignored
        )
        XCTAssertEqual(
            recorder.recordFrame(makeFrame(timestamp: 100.2, frameNumber: 3), for: active),
            .recorded
        )
        XCTAssertEqual(recorder.recordedFrameCount, 1)
        XCTAssertEqual(recorder.frames.first?.frameNumber, 3)
    }

    func testRealCaptureDurationLimitUsesButtonPressOrigin() {
        let recorder = MotionRecorder()
        let epoch = CaptureEpoch(timeOrigin: 100)
        XCTAssertTrue(recorder.startRecording(for: epoch))

        XCTAssertEqual(
            recorder.recordFrame(makeFrame(timestamp: 159, frameNumber: 1), for: epoch),
            .recorded
        )
        XCTAssertEqual(recorder.duration, 59, accuracy: 0.001)
        XCTAssertEqual(
            recorder.recordFrame(makeFrame(timestamp: 160, frameNumber: 2), for: epoch),
            .recordedAndStoppedAtLimit
        )
        XCTAssertEqual(recorder.duration, 60, accuracy: 0.001)
        XCTAssertFalse(recorder.isRecording)
    }

    func testAverageFPS() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        // 61 frames over 1 second = 60 fps
        for i in 0...60 {
            recorder.recordFrame(makeFrame(timestamp: Double(i) / 60.0, frameNumber: i + 1))
        }

        XCTAssertEqual(recorder.averageFPS, 60.0, accuracy: 0.1)
    }

    func testAverageFPSSingleFrame() {
        let recorder = MotionRecorder()
        recorder.startRecording()
        recorder.recordFrame(makeFrame(timestamp: 0.0, frameNumber: 1))
        XCTAssertEqual(recorder.averageFPS, 0)
    }

    func testDurationWithOffset() {
        let recorder = MotionRecorder()
        recorder.startRecording()

        // Timestamps don't start at zero (simulating real ARKit timestamps)
        recorder.recordFrame(makeFrame(timestamp: 100.0, frameNumber: 1))
        recorder.recordFrame(makeFrame(timestamp: 100.5, frameNumber: 2))
        recorder.recordFrame(makeFrame(timestamp: 101.0, frameNumber: 3))

        XCTAssertEqual(recorder.duration, 1.0, accuracy: 0.001)
    }

    func testContentViewWiresAtomicLifecycleAndDetachedExport() throws {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let source = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/App/ContentView.swift"
            ),
            encoding: .utf8
        )

        XCTAssertTrue(source.contains("@Environment(\\.scenePhase)"))
        XCTAssertTrue(source.contains("capture.process(frame"))
        XCTAssertTrue(source.contains("reason: .trackingLost"))
        XCTAssertTrue(source.contains("reason: .offlineImport"))
        XCTAssertTrue(source.contains("reason: .appInactive"))
        XCTAssertTrue(source.contains(".confirmationDialog("))
        XCTAssertTrue(source.contains("CaptureExportSnapshot("))
        XCTAssertTrue(source.contains("Task.detached(priority: .userInitiated)"))
        XCTAssertTrue(source.contains("completionWithItemsHandler"))
        XCTAssertTrue(source.contains("ShareCompletionPolicy.mayMarkExported"))
        XCTAssertTrue(source.contains("let sharingEpoch = exportedEpoch"))
        XCTAssertTrue(source.contains("capture.markExported(epoch: sharingEpoch)"))
        XCTAssertFalse(source.contains("capture.markExported(epoch: exportedEpoch)"))
        XCTAssertTrue(source.contains("outcome.hasMotionArtifact ? snapshot.epoch : nil"))
        XCTAssertTrue(source.contains(".accessibilityLabel(recorder.isRecording"))
        XCTAssertFalse(source.contains("recorder.startRecording()"))
        XCTAssertFalse(source.contains("recorder.stopRecording()"))
        XCTAssertFalse(source.contains("nimble.startRecordingResults()"))
        XCTAssertFalse(source.contains("nimble.stopRecordingResults()"))

        let trackingStart = try XCTUnwrap(source.range(
            of: ".onChange(of: bodyTracking.isTracking)"
        ))
        let sceneStart = try XCTUnwrap(source.range(
            of: ".onChange(of: scenePhase)",
            range: trackingStart.upperBound..<source.endIndex
        ))
        let trackingHandler = source[trackingStart.lowerBound..<sceneStart.lowerBound]
        let trackingStop = try XCTUnwrap(trackingHandler.range(of: "reason: .trackingLost"))
        let trackingReset = try XCTUnwrap(
            trackingHandler.range(of: "nimble.resetSessionState()")
        )
        XCTAssertLessThan(trackingStop.lowerBound, trackingReset.lowerBound)

        let offlineStart = try XCTUnwrap(source.range(
            of: ".onChange(of: showOfflineImport)"
        ))
        let offlineEnd = try XCTUnwrap(source.range(
            of: ".sheet(isPresented: $showOfflineImport)",
            range: offlineStart.upperBound..<source.endIndex
        ))
        let offlineHandler = source[offlineStart.lowerBound..<offlineEnd.lowerBound]
        let offlineStop = try XCTUnwrap(offlineHandler.range(of: "reason: .offlineImport"))
        let offlineReset = try XCTUnwrap(offlineHandler.range(of: "nimble.resetSessionState()"))
        XCTAssertLessThan(offlineStop.lowerBound, offlineReset.lowerBound)
    }

    func testShareCompletionOnlyMarksExportedAfterAnErrorFreeActivity() {
        XCTAssertTrue(ShareCompletionPolicy.mayMarkExported(
            completed: true,
            activityError: nil
        ))
        XCTAssertFalse(ShareCompletionPolicy.mayMarkExported(
            completed: false,
            activityError: nil
        ))
        XCTAssertFalse(ShareCompletionPolicy.mayMarkExported(
            completed: true,
            activityError: NSError(domain: "BioMotionTests", code: 1)
        ))
    }

    @MainActor
    func testCoordinatorStartsAndStopsMarkerAndSolverAsOneEpoch() async {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()

        let result = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 42
        )
        guard case let .started(epoch) = result else {
            return XCTFail("expected an atomic capture start, got \(result)")
        }
        XCTAssertEqual(epoch.timeOrigin, 42)
        XCTAssertEqual(recorder.captureEpoch, epoch)
        XCTAssertEqual(engine.resultHistoryEpoch, epoch)
        XCTAssertTrue(recorder.isRecording)
        XCTAssertTrue(engine.recordingResultsAreArmed)

        coordinator.stop(recorder: recorder, nimble: engine, reason: .user)
        XCTAssertFalse(recorder.isRecording)
        XCTAssertFalse(engine.recordingResultsAreArmed)
        XCTAssertNil(coordinator.activeEpoch)
    }

    @MainActor
    func testCoordinatorBlocksUnexportedReplacementButAllowsExplicitDiscard() async {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()
        guard case let .started(firstEpoch) = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 10
        ) else { return XCTFail("first capture did not start") }
        XCTAssertEqual(firstEpoch.timeOrigin, 10)
        recorder.recordFrame(
            makeFrame(timestamp: 10.1, frameNumber: 1),
            for: coordinator.activeEpoch
        )
        coordinator.stop(recorder: recorder, nimble: engine, reason: .user)

        XCTAssertTrue(coordinator.hasUnexportedCapture)
        XCTAssertEqual(
            coordinator.start(recorder: recorder, nimble: engine, timeOrigin: 20),
            .blockedByUnexportedCapture
        )
        XCTAssertEqual(recorder.recordedFrameCount, 1)

        guard case let .started(secondEpoch) = coordinator.discardAndStart(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 20
        ) else { return XCTFail("replacement capture did not start") }
        XCTAssertEqual(secondEpoch.timeOrigin, 20)
        XCTAssertNotEqual(firstEpoch.id, secondEpoch.id)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertFalse(coordinator.hasUnexportedCapture)
    }

    @MainActor
    func testCoordinatorRefusesCaptureUntilModelIsLoaded() {
        let result = LiveRecordingCoordinator().start(
            recorder: MotionRecorder(),
            nimble: NimbleEngine(),
            timeOrigin: 10
        )
        XCTAssertEqual(result, .unavailable)
    }

    @MainActor
    func testOnlyMatchingExportCompletionClearsUnexportedGuard() async {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()

        guard case let .started(firstEpoch) = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 10
        ) else { return XCTFail("first capture did not start") }
        coordinator.process(
            makeFrame(timestamp: 10.1, frameNumber: 1),
            recorder: recorder,
            nimble: engine
        )
        coordinator.stop(recorder: recorder, nimble: engine, reason: .user)

        guard case let .started(secondEpoch) = coordinator.discardAndStart(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 20
        ) else { return XCTFail("replacement capture did not start") }
        XCTAssertNotEqual(firstEpoch.id, secondEpoch.id)
        coordinator.process(
            makeFrame(timestamp: 20.1, frameNumber: 2),
            recorder: recorder,
            nimble: engine
        )
        coordinator.stop(recorder: recorder, nimble: engine, reason: .user)

        coordinator.markExported(epoch: firstEpoch)
        XCTAssertTrue(coordinator.hasUnexportedCapture)
        coordinator.markExported(epoch: secondEpoch)
        XCTAssertFalse(coordinator.hasUnexportedCapture)

        // A later lifecycle callback is not a new capture boundary and must
        // not resurrect an already-exported take.
        coordinator.stop(recorder: recorder, nimble: engine, reason: .appInactive)
        XCTAssertFalse(coordinator.hasUnexportedCapture)
    }

    @MainActor
    func testStopFreezesCaptureBeforeAResetCanClearEngineHistory() async throws {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()
        guard case let .started(epoch) = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 100
        ) else { return XCTFail("capture did not start") }

        coordinator.process(
            makeFrame(timestamp: 100.1, frameNumber: 1),
            recorder: recorder,
            nimble: engine
        )
        coordinator.stop(recorder: recorder, nimble: engine, reason: .trackingLost)
        let frozen = try XCTUnwrap(coordinator.completedSnapshot)

        XCTAssertTrue(engine.resetSessionState())
        XCTAssertEqual(frozen.epoch, epoch)
        XCTAssertEqual(frozen.frames.map(\.frameNumber), [1])
        XCTAssertEqual(coordinator.completedSnapshot?.epoch, epoch)
    }

    @MainActor
    func testDurationLimitStopsMarkerAndSolverCaptureTogether() async {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()
        guard case let .started(epoch) = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 100
        ) else { return XCTFail("capture did not start") }

        coordinator.process(
            makeFrame(timestamp: 160, frameNumber: 1),
            recorder: recorder,
            nimble: engine
        )

        XCTAssertFalse(recorder.isRecording)
        XCTAssertFalse(engine.recordingResultsAreArmed)
        XCTAssertNil(coordinator.activeEpoch)
        XCTAssertEqual(coordinator.lastStopReason, .frameLimit)
        XCTAssertEqual(coordinator.completedSnapshot?.epoch, epoch)
    }

    @MainActor
    func testEngineResetStopsMarkerCaptureBeforeTheNextFrame() async {
        let recorder = MotionRecorder()
        let engine = await loadedEngine()
        let coordinator = LiveRecordingCoordinator()
        guard case .started = coordinator.start(
            recorder: recorder,
            nimble: engine,
            timeOrigin: 100
        ) else { return XCTFail("capture did not start") }

        XCTAssertTrue(engine.resetRealtimeState())
        XCTAssertFalse(engine.recordingResultsAreArmed)
        coordinator.process(
            makeFrame(timestamp: 100.1, frameNumber: 1),
            recorder: recorder,
            nimble: engine
        )

        XCTAssertFalse(recorder.isRecording)
        XCTAssertEqual(recorder.recordedFrameCount, 0)
        XCTAssertNil(coordinator.activeEpoch)
        XCTAssertEqual(coordinator.lastStopReason, .solverReset)
    }
}
