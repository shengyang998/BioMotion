import simd
import UIKit
import XCTest
@testable import BioMotion

/// Drives `NimbleEngine` the way `OfflineSessionRunner` does — one frame at a
/// time, waiting for each publish before submitting the next — and checks that
/// centred pose routing remains live while the bundled model's missing contact
/// support keeps every contact-dependent result fail-closed.
final class OfflineOrchestrationTests: XCTestCase {

    private var engine: NimbleEngine!

    override func setUp() {
        super.setUp()
        engine = NimbleEngine()
    }

    override func tearDown() {
        engine = nil
        super.tearDown()
    }

    private func bodyFrame(timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        let joints = OfflineMuscleChainFixture.markers.map { arkitId, opensimMarker, p in
            TrackedJoint(id: arkitId, name: arkitId, worldPosition: p, isTracked: true,
                         opensimMarkerNameOverride: opensimMarker)
        }
        return BodyFrame(
            timestamp: timestamp,
            frameNumber: frameNumber,
            joints: joints,
            dynamicsReference: .mhrRootRelative
        )
    }

    /// How long one submission may take before this test calls it dead.
    ///
    /// It is a LIVENESS bound, not a performance budget — this suite exists to
    /// isolate the async orchestration layer, and a Debug simulator build is not
    /// where per-frame cost is judged. It was an unnamed `= 10` default until
    /// 2026-08-08, when ellipsoid path wrapping made the ninth submission — the
    /// first one where the Savitzky-Golay window is full, so the first that runs
    /// ID + moment arms + QP — cross it, and the suite failed with
    /// "no muscle output" rather than with the cost that caused it. The number
    /// below has a measurement behind it and the per-push seconds are printed,
    /// so a real regression shows up as a number instead of as a mystery.
    ///
    /// Measured on this machine, Debug, iOS Simulator, `FullBody.osim`: pushes
    /// 1–7 cost **0.09 s** each (IK only — the window is not full), push 0 costs
    /// **4.07 s** (first solve, cold), and push 8 — the first with a full window,
    /// so the first that runs ID + moment arms + QP — costs **11.65 s**. Before
    /// the ellipsoids the whole test ran in 12.45–12.56 s with every submission
    /// inside the old 10 s bound, so push 8 was under 10 s and is now over it.
    /// 45 s is ~4× the measured worst case, and `timedOut == 0` is asserted, so a
    /// large regression still fails — with the seconds printed beside it.
    private let submissionLivenessTimeout: TimeInterval = 45

    /// Mirrors `OfflineSessionRunner.submitAndWait`: subscribe first, submit,
    /// then wait for the engine to publish.
    @MainActor
    private func submitAndWait(_ frame: BodyFrame, timeout: TimeInterval) async -> Bool {
        switch await NimbleFrameWaiter.submit(on: engine, timeout: timeout, {
            engine.processFrame(frame)
        }) {
        case .published: return true
        case .failed, .timedOut, .superseded, .dropped, .rejected:
            return false
        }
    }

    @MainActor
    private func loadEngineForGroundTrustTest() async {
        engine.loadBundledModel()
        let deadline = Date().addingTimeInterval(60)
        while !engine.isModelLoaded && Date() < deadline {
            try? await Task.sleep(nanoseconds: 100_000_000)
        }
        XCTAssertTrue(engine.isModelLoaded, "model never finished loading")
    }

    /// Ground trust currently lives inside the bridge, while the contract being
    /// exercised here is the engine's publication boundary. Reflection keeps
    /// this RED test test-only until that boundary has a production seam of its
    /// own; it is deliberately limited to seeding and observing trust state.
    private func groundBridge() throws -> NimbleBridge {
        let child = Mirror(reflecting: engine as NimbleEngine).children.first {
            $0.label == "bridge"
        }
        return try XCTUnwrap(child?.value as? NimbleBridge,
                             "NimbleEngine no longer exposes its bridge to the ground-trust test seam")
    }

    @MainActor
    private func submitConstantFrames(count: Int,
                                      startingAt start: TimeInterval = 0,
                                      frameNumberBase: Int = 0) async {
        let dt = 1.0 / 30.0
        for index in 0..<count {
            let published = await submitAndWait(
                bodyFrame(timestamp: start + Double(index) * dt,
                          frameNumber: frameNumberBase + index),
                timeout: submissionLivenessTimeout)
            XCTAssertTrue(published, "ground-trust push \(index) did not publish")
        }
    }

    private func assertNoRecordedIDHistory(file: StaticString = #filePath,
                                           line: UInt = #line) {
        let filename = "GroundTrust_untrusted_id"
        do {
            let url = try engine.exportSTO(filename: filename)
            try? FileManager.default.removeItem(at: url)
            XCTFail("untrusted ground leaked inverse dynamics into ID history",
                    file: file, line: line)
        } catch NimbleEngine.ExportError.noData {
            // The public export boundary is the observable ID-history contract.
        } catch {
            XCTFail("unexpected ID-history export error: \(error)", file: file, line: line)
        }
    }

    /// Reset is allowed while an old solve is still on the serial queue. Every
    /// mutable side effect that solve can reach before its main-thread publish
    /// must therefore either share the queue or sit behind the generation
    /// guard. This source contract pins the ownership boundary that makes the
    /// otherwise timing-dependent race deterministic to review.
    func testResetOwnedBuffersAreQueueAndGenerationConfined() throws {
        let source = try nimbleEngineSource()
        let resetStart = try XCTUnwrap(source.range(of: "    func resetRealtimeState("))
        let resetEnd = try XCTUnwrap(
            source.range(of: "    func resetSessionState(", range: resetStart.upperBound..<source.endIndex))
        let resetBody = String(source[resetStart.lowerBound..<resetEnd.lowerBound])
        let queuedResetEnd = try XCTUnwrap(
            resetBody.range(of: "\n        }\n\n        lastDisplayMuscleTimestamp"))
        let queuedReset = String(resetBody[..<queuedResetEnd.upperBound])
        XCTAssertTrue(
            queuedReset.contains("activationFilters.removeAll(keepingCapacity: false)"),
            "display filters are solverQueue-owned and must be cleared on that same queue"
        )
        XCTAssertTrue(queuedReset.contains("if resetsBridgeSession"))
        XCTAssertTrue(queuedReset.contains("if resetsMuscleSession"),
                      "clip/pass resets must be in the queued block before notifications")
        let queuedBlock = try XCTUnwrap(resetBody.range(of: "solverQueue.async"))
        let notification = try XCTUnwrap(resetBody.range(
            of: "frameCompletionSubject.send(FrameCompletion("))
        XCTAssertLessThan(queuedBlock.lowerBound, notification.lowerBound,
                          "a synchronous subscriber cannot enqueue between reset scopes")

        XCTAssertEqual(
            source.components(separatedBy: "ikHistory.append").count - 1,
            1,
            "history must have one main-thread publication owner, not solverQueue writers"
        )
        let publishStart = try XCTUnwrap(source.range(of: "    private func publishResults("))
        let publishEnd = try XCTUnwrap(
            source.range(of: "    func resetRealtimeState(", range: publishStart.upperBound..<source.endIndex))
        let publication = String(source[publishStart.lowerBound..<publishEnd.lowerBound])
        let guardIndex = try XCTUnwrap(
            publication.range(of: "guard self.readGeneration() == generation else {"))
        let historyIndex = try XCTUnwrap(publication.range(of: "self.ikHistory.append"))
        XCTAssertLessThan(guardIndex.lowerBound, historyIndex.lowerBound,
                          "a discarded generation must not enter recording history")

        let idHistoryIndex = try XCTUnwrap(publication.range(of: "self.idHistory.append"))
        let idProvenanceIndex = try XCTUnwrap(publication.range(
            of: "Self.inverseDynamicsPayloadIsSameGeneration("))
        XCTAssertLessThan(idProvenanceIndex.lowerBound, idHistoryIndex.lowerBound,
                          "ID history must verify its IK/ID timestamps before relabelling the row")
    }

    /// Regression for the live/offline recording seam: acquiring the offline
    /// policy resets the shared solver, but it used to leave result recording
    /// armed. Offline IK rows then entered the live MOT/STO history while the
    /// marker recorder correctly rejected those same offline frames.
    @MainActor
    func testOfflinePolicyAcquisitionDisarmsLiveResultRecording() throws {
        engine.startRecordingResults()
        XCTAssertTrue(try resultRecordingIsArmed())

        let lease = engine.acquireOfflinePolicyLease()
        defer { _ = engine.releaseOfflinePolicyLease(lease) }

        XCTAssertFalse(
            try resultRecordingIsArmed(),
            "an offline owner must never inherit the live capture's IK/ID recording flag"
        )
    }

    /// A full AR world/session reset is a recording boundary. Leaving result
    /// recording armed mixes rows expressed in two unrelated world origins.
    @MainActor
    func testSessionResetDisarmsLiveResultRecording() throws {
        engine.startRecordingResults()
        XCTAssertTrue(try resultRecordingIsArmed())

        XCTAssertTrue(engine.resetSessionState())

        XCTAssertFalse(
            try resultRecordingIsArmed(),
            "a reset world origin must end the capture before another frame can publish"
        )
    }

    /// Solver work is asynchronous: a frame admitted during capture A may
    /// publish after the user has stopped A and started B. Generation alone
    /// cannot reject it because stop/start is not an AR/session reset. Pin a
    /// submission-time capture epoch through the receipt and require the same
    /// epoch at the one history publication owner.
    func testFrameReceiptFencesLateResultsAcrossCaptures() throws {
        let epochA = CaptureEpoch(timeOrigin: 100)
        let receipt = NimbleEngine.FrameReceipt(
            generation: 4,
            submissionID: 18,
            captureEpoch: epochA
        )
        XCTAssertEqual(receipt.captureEpoch, epochA)

        let source = try nimbleEngineSource()
        let processStart = try XCTUnwrap(source.range(of: "    func processFrame("))
        let processEnd = try XCTUnwrap(
            source.range(of: "    private func publishResults(",
                         range: processStart.upperBound..<source.endIndex)
        )
        let process = String(source[processStart.lowerBound..<processEnd.lowerBound])
        let preOriginGate = try XCTUnwrap(
            process.range(of: "RecordingCapturePolicy.mayProcessFrame(")
        )
        let solverSideEffect = try XCTUnwrap(process.range(of: "isFrameInFlight = true"))
        XCTAssertLessThan(
            preOriginGate.lowerBound,
            solverSideEffect.lowerBound,
            "a queued pre-origin live frame must fail before solver/filter state changes"
        )
        XCTAssertTrue(
            process.contains("RecordingCapturePolicy.epochForSubmission("),
            "submission admission must use the behavior-tested capture policy"
        )

        let publishStart = try XCTUnwrap(source.range(of: "    private func publishResults("))
        let publishEnd = try XCTUnwrap(
            source.range(of: "    func resetRealtimeState(",
                         range: publishStart.upperBound..<source.endIndex)
        )
        let publication = String(source[publishStart.lowerBound..<publishEnd.lowerBound])
        XCTAssertTrue(
            publication.contains("RecordingCapturePolicy.mayPublish("),
            "the history owner must use the behavior-tested capture policy"
        )
    }

    func testCapturePolicyRejectsLateAndOfflineResultsByValue() {
        let epochA = CaptureEpoch(timeOrigin: 100)
        let epochB = CaptureEpoch(timeOrigin: 200)

        XCTAssertFalse(RecordingCapturePolicy.mayProcessFrame(
            activeEpoch: epochA,
            frameTimestamp: 99.999,
            isOfflineSubmission: false
        ), "a queued pre-origin live frame must not enter native temporal state")
        XCTAssertTrue(RecordingCapturePolicy.mayProcessFrame(
            activeEpoch: epochA,
            frameTimestamp: 100,
            isOfflineSubmission: false
        ))
        XCTAssertTrue(RecordingCapturePolicy.mayProcessFrame(
            activeEpoch: epochA,
            frameTimestamp: 1,
            isOfflineSubmission: true
        ), "offline analysis keeps its independent timestamp domain")
        XCTAssertTrue(RecordingCapturePolicy.mayProcessFrame(
            activeEpoch: nil,
            frameTimestamp: 1,
            isOfflineSubmission: false
        ), "normal unrecorded live tracking remains available")

        let admittedA = RecordingCapturePolicy.epochForSubmission(
            activeEpoch: epochA,
            frameTimestamp: 100.1,
            isOfflineSubmission: false
        )
        XCTAssertEqual(admittedA, epochA)
        XCTAssertNil(RecordingCapturePolicy.epochForSubmission(
            activeEpoch: epochA,
            frameTimestamp: 100.1,
            isOfflineSubmission: true
        ))
        XCTAssertNil(RecordingCapturePolicy.epochForSubmission(
            activeEpoch: epochA,
            frameTimestamp: 99.999,
            isOfflineSubmission: false
        ), "the solver must reject the same pre-origin frame as the marker recorder")

        XCTAssertFalse(RecordingCapturePolicy.mayPublish(
            submissionEpoch: admittedA,
            activeEpoch: epochB,
            isArmed: true,
            hasMotion: true
        ), "capture A's late result must not enter capture B")
        XCTAssertTrue(RecordingCapturePolicy.mayPublish(
            submissionEpoch: admittedA,
            activeEpoch: epochA,
            isArmed: true,
            hasMotion: true
        ))
        XCTAssertFalse(RecordingCapturePolicy.mayPublish(
            submissionEpoch: admittedA,
            activeEpoch: epochA,
            isArmed: false,
            hasMotion: true
        ))
        XCTAssertFalse(RecordingCapturePolicy.mayPublish(
            submissionEpoch: nil,
            activeEpoch: epochA,
            isArmed: true,
            hasMotion: true
        ))
    }

    private func resultRecordingIsArmed() throws -> Bool {
        let child = Mirror(reflecting: engine as NimbleEngine).children.first {
            $0.label == "isRecordingResults"
        }
        return try XCTUnwrap(
            child?.value as? Bool,
            "NimbleEngine no longer exposes the result-recording ownership seam"
        )
    }

    private func nimbleEngineSource() throws -> String {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        return try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Nimble/NimbleEngine.swift"),
            encoding: .utf8)
    }

    @MainActor
    func testTrustedGroundStillCannotUnlockMissingContactSupport() async throws {
        await loadEngineForGroundTrustTest()
        let bridge = try groundBridge()

        for _ in 0..<30 { bridge.observeLowestFootHeightY(0) }
        XCTAssertEqual(bridge.groundHeightSource, .estimated)
        XCTAssertTrue(bridge.groundHeightTrusted)
        XCTAssertFalse(bridge.hasValidatedFootContactSupport)

        engine.startRecordingResults()
        defer { engine.stopRecordingResults() }
        await submitConstantFrames(count: SavitzkyGolayFilter.windowSize)

        let rejected = try XCTUnwrap(engine.lastSolve,
                                     "the warm solve should still publish its centred pose")
        XCTAssertEqual(rejected.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(rejected.id?.timestamp)
        XCTAssertNil(rejected.muscle?.timestamp)
        XCTAssertNil(rejected.gait?.residualInBodyWeights,
                     "trusted ground is necessary but cannot create contact mechanics")
        XCTAssertNil(engine.lastIDResult?.timestamp)
        XCTAssertNil(engine.lastMuscleResult?.timestamp)
        assertNoRecordedIDHistory()

        // Nor may the explicit-floor seam bypass the model/solver capability.
        engine.setExplicitGroundHeightY(0)
        await submitConstantFrames(count: 1,
                                   startingAt: Double(SavitzkyGolayFilter.windowSize) / 30.0,
                                   frameNumberBase: SavitzkyGolayFilter.windowSize)
        XCTAssertTrue(bridge.groundHeightTrusted)
        XCTAssertEqual(engine.lastSolve?.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(engine.lastSolve?.id)
        XCTAssertNil(engine.lastSolve?.muscle)
        assertNoRecordedIDHistory()
    }

    @MainActor
    func testUnsupportedContactGaitKeepsTimingButHasNoLoadSummary() async throws {
        await loadEngineForGroundTrustTest()
        engine.setExplicitGroundHeightY(0)
        let dt = 1.0 / 30.0
        let taps = WindowedDerivativeFilter.minimumTaps
        engine.gaitPlan = NimbleEngine.GaitPlan(
            frames: (0..<taps).map { index in
                NimbleEngine.GaitPlan.Frame(
                    timestamp: Double(index) * dt,
                    verticalForceInBodyWeights: 2,
                    contactSide: -1,
                    contactIndex: 0,
                    derivativeWindowInsideContact: true)
            },
            filterTaps: taps,
            sampleInterval: dt)

        await submitConstantFrames(count: taps)

        let solve = try XCTUnwrap(engine.lastSolve,
                                  "the warm gait solve should still publish its centred pose")
        XCTAssertEqual(solve.motion.verdict, .gaitStance)
        XCTAssertEqual(solve.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(solve.id?.timestamp)
        XCTAssertNil(solve.muscle?.timestamp)
        XCTAssertNil(solve.gait?.residualInBodyWeights,
                     "kinematic stance timing cannot become a gait-load outcome")

        let frame = OfflineResultStore.FrameResult(
            id: 0,
            sourceImage: UIImage(),
            timestamp: solve.centerTimestamp,
            status: .success,
            usedFallbackBBox: false,
            camT: nil,
            modelChecksums: nil,
            bodyFrame: nil,
            ikResult: solve.ik,
            idResult: solve.id,
            muscleResult: solve.muscle,
            dynamicsAvailability: solve.dynamicsAvailability,
            isStaticHoldEstimate: solve.isStaticHoldEstimate,
            motionState: .gait(verdict: solve.motion.verdict, outcome: solve.gait))
        let fixture = try GaitClipFixture.load("video_012", bundle: Bundle(for: type(of: self)))
        let report = try GaitAnalysis.analyse(frames: fixture.frames)
        let summary = GaitLoadSummary.make(frames: [frame], report: report, filterTaps: taps)
        XCTAssertNil(summary,
                     "missing contact support must not manufacture a gait-load summary")
    }

    @MainActor
    func testSessionResetClearsPoseOnlyStateWithoutChangingContactCapability() async throws {
        await loadEngineForGroundTrustTest()
        let bridge = try groundBridge()

        for _ in 0..<30 { bridge.observeLowestFootHeightY(0) }
        await submitConstantFrames(count: SavitzkyGolayFilter.windowSize)
        XCTAssertTrue(bridge.groundHeightTrusted)
        XCTAssertFalse(bridge.hasValidatedFootContactSupport)
        XCTAssertEqual(engine.lastSolve?.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(engine.lastSolve?.id)
        XCTAssertNil(engine.lastSolve?.muscle)

        engine.resetSessionState()
        XCTAssertNil(engine.lastSolve)
        XCTAssertNil(engine.lastIDResult?.timestamp)
        XCTAssertNil(engine.lastMuscleResult?.timestamp)

        await submitConstantFrames(count: SavitzkyGolayFilter.windowSize,
                                   startingAt: 10,
                                   frameNumberBase: 100)

        XCTAssertEqual(bridge.groundHeightSource, .uncalibrated,
                       "the rejected production path must not refill the floor estimator")
        XCTAssertFalse(bridge.groundHeightTrusted,
                       "a new session must not inherit the old floor")
        XCTAssertFalse(bridge.hasValidatedFootContactSupport,
                       "session reset does not change model/solver capability")
        let restarted = try XCTUnwrap(engine.lastSolve,
                                      "the new session should still publish a centred pose")
        XCTAssertEqual(restarted.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(restarted.id?.timestamp)
        XCTAssertNil(restarted.muscle?.timestamp)
        XCTAssertNil(restarted.gait?.residualInBodyWeights)
        XCTAssertNil(engine.lastIDResult?.timestamp)
        XCTAssertNil(engine.lastMuscleResult?.timestamp)
    }

    @MainActor
    func testNineSubmissionsPublishCentredPoseWithoutContactDynamics() async throws {
        engine.loadBundledModel()

        // Model load is async with only `isModelLoaded` as a signal.
        let deadline = Date().addingTimeInterval(60)
        while !engine.isModelLoaded && Date() < deadline {
            try? await Task.sleep(nanoseconds: 100_000_000)
        }
        XCTAssertTrue(engine.isModelLoaded, "model never finished loading")
        // An explicit floor isolates async routing while proving that floor
        // calibration cannot bypass missing foot-support mechanics.
        engine.setExplicitGroundHeightY(0)

        // The offline runner's cadence: 4 head-pad + 1 real + 4 tail-pad, all
        // at the same pose, spaced on the clip's sample interval.
        let dt = 1.0 / 30.0
        var published = 0
        var timedOut = 0
        var elapsed: [Double] = []
        for push in 0..<SavitzkyGolayFilter.windowSize {
            let ts = Double(push - SavitzkyGolayFilter.halfWindow) * dt
            let start = Date()
            if await submitAndWait(bodyFrame(timestamp: ts, frameNumber: push),
                                   timeout: submissionLivenessTimeout) {
                published += 1
            } else {
                timedOut += 1
            }
            elapsed.append(Date().timeIntervalSince(start))
            print(String(format: "ORCH-METRIC push=%d ts=%.4f muscle=%@ dropped=%d elapsed=%.2f s",
                         push, ts, engine.lastMuscleResult != nil ? "true" : "false",
                         engine.droppedFrameCount, elapsed[push]))
        }

        let slowest = elapsed.max() ?? 0
        print(String(format: "ORCH-METRIC published=%d timedOut=%d dropped=%d slowest=%.2f s total=%.2f s",
                     published, timedOut, engine.droppedFrameCount, slowest, elapsed.reduce(0, +)))
        XCTAssertEqual(timedOut, 0,
                       "a submission never published within \(submissionLivenessTimeout) s — "
                       + "slowest \(slowest) s. That is a liveness failure or a very large "
                       + "cost regression, not a small one")
        XCTAssertEqual(engine.droppedFrameCount, 0,
                       "frames were dropped — the runner submitted while a solve was in flight")

        let solve = try XCTUnwrap(engine.lastSolve,
                                  "no centred solve after \(SavitzkyGolayFilter.windowSize) submissions")
        XCTAssertEqual(solve.centerTimestamp, 0.0, accuracy: 1e-6,
                       "the publication belongs to the middle push, not the newest one")
        XCTAssertEqual(solve.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(solve.id)
        XCTAssertNil(solve.muscle)
        XCTAssertNil(solve.gait)
        XCTAssertNil(engine.lastIDResult)
        XCTAssertNil(engine.lastMuscleResult)

        // A temporal gap clears SG/hold/display state but keeps the clip's IK
        // warm start. The reset happens BEFORE the next waiter subscribes; if
        // its own `objectWillChange` were mistaken for a solve, the IK timestamp
        // below would stay nil/stale and this loop would fail immediately.
        engine.resetRealtimeState()
        XCTAssertNil(engine.lastSolve)

        let resetStart: TimeInterval = 10
        for push in 0..<(SavitzkyGolayFilter.windowSize - 1) {
            let timestamp = resetStart + Double(push) * dt
            let published = await submitAndWait(
                bodyFrame(timestamp: timestamp, frameNumber: 100 + push),
                timeout: submissionLivenessTimeout)
            XCTAssertTrue(published, "post-gap push \(push) did not publish")
            XCTAssertEqual(engine.lastIKResult?.timestamp ?? -Double.infinity,
                           timestamp,
                           accuracy: 1e-6,
                           "the waiter resumed on reset rather than this frame")
            XCTAssertNil(engine.lastSolve,
                         "a derivative solve appeared before the reset window refilled")
        }

        let finalPush = SavitzkyGolayFilter.windowSize - 1
        let finalTimestamp = resetStart + Double(finalPush) * dt
        let finalPublished = await submitAndWait(
            bodyFrame(timestamp: finalTimestamp, frameNumber: 100 + finalPush),
            timeout: submissionLivenessTimeout)
        XCTAssertTrue(finalPublished)
        let postGapSolve = try XCTUnwrap(engine.lastSolve,
                                         "the Tth trusted push must refill the window")
        XCTAssertEqual(postGapSolve.centerTimestamp,
                       resetStart + Double(SavitzkyGolayFilter.halfWindow) * dt,
                       accuracy: 1e-6)
        XCTAssertEqual(postGapSolve.dynamicsAvailability, .contactSupportUnavailable)
        XCTAssertNil(postGapSolve.id)
        XCTAssertNil(postGapSolve.muscle)
        XCTAssertEqual(engine.droppedFrameCount, 0,
                       "reset and resubmission must preserve offline backpressure")
    }
}

/// Shared so `OfflineOrchestrationTests` and `OfflineMuscleChainTests` cannot
/// drift apart on the pose they exercise.
enum OfflineMuscleChainFixture {
    static let markers: [(String, String, SIMD3<Float>)] = [
        ("hips_joint", "MHR_ROOT", SIMD3<Float>(0.000000, 0.923987, 0.000000)),
        ("left_upLeg_joint", "LHJC", SIMD3<Float>(0.049532, 0.940746, -0.059429)),
        ("right_upLeg_joint", "RHJC", SIMD3<Float>(-0.026827, 0.888276, 0.065355)),
        ("left_leg_joint", "LKJC", SIMD3<Float>(0.381692, 1.159680, 0.103828)),
        ("right_leg_joint", "RKJC", SIMD3<Float>(-0.016599, 0.460764, 0.110560)),
        ("left_foot_joint", "LAJC", SIMD3<Float>(0.271441, 0.770530, 0.026491)),
        ("right_foot_joint", "RAJC", SIMD3<Float>(-0.161081, 0.080435, 0.046929)),
        ("left_toes_joint", "LTOE", SIMD3<Float>(0.307562, 0.632544, 0.055199)),
        ("right_toes_joint", "RTOE", SIMD3<Float>(-0.106420, -0.043160, 0.104657)),
        ("spine_1_joint", "SPINE_L", SIMD3<Float>(-0.089250, 1.038710, -0.005759)),
        ("spine_4_joint", "SPINE_M", SIMD3<Float>(-0.240898, 1.231510, 0.003535)),
        ("spine_7_joint", "C7", SIMD3<Float>(-0.304259, 1.356519, 0.044218)),
        ("neck_1_joint", "NECK", SIMD3<Float>(-0.319732, 1.374271, 0.044922)),
        ("head_joint", "HEAD", SIMD3<Float>(-0.446991, 1.441565, 0.027405)),
        ("left_shoulder_1_joint", "LSJC", SIMD3<Float>(-0.274981, 1.417246, -0.078846)),
        ("right_shoulder_1_joint", "RSJC", SIMD3<Float>(-0.372616, 1.261433, 0.128290)),
        ("left_forearm_joint", "LEJC", SIMD3<Float>(-0.336363, 1.682226, -0.073540)),
        ("right_forearm_joint", "REJC", SIMD3<Float>(-0.424209, 0.995907, 0.157344)),
        ("left_hand_joint", "LWJC", SIMD3<Float>(-0.577720, 1.737568, 0.012645)),
        ("right_hand_joint", "RWJC", SIMD3<Float>(-0.457117, 0.739004, 0.198106)),
    ]
}

private extension Float {
    var rounded3: Float { (self * 1000).rounded() / 1000 }
}
