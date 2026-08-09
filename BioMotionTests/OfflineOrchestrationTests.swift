import simd
import XCTest
@testable import BioMotion

/// Drives `NimbleEngine` the way `OfflineSessionRunner` does — one frame at a
/// time, waiting for each publish before submitting the next — and checks that
/// muscle output actually appears.
///
/// `OfflineMuscleChainTests` already proves the biomechanics chain itself works
/// on this exact pose (IK -> SG -> ID -> moment arms -> QP all succeed). So if
/// the app still reports "0 with muscle data", the fault is in this async
/// orchestration layer, not in the solver. This test isolates that layer.
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
        let joints = OfflineMuscleChainFixture.markers.map { arkitId, _, p in
            TrackedJoint(id: arkitId, name: arkitId, worldPosition: p, isTracked: true)
        }
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
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
        case .timedOut, .dropped: return false
        }
    }

    @MainActor
    func testNineSubmissionsProduceMuscleOutput() async throws {
        engine.loadBundledModel()

        // Model load is async with only `isModelLoaded` as a signal.
        let deadline = Date().addingTimeInterval(60)
        while !engine.isModelLoaded && Date() < deadline {
            try? await Task.sleep(nanoseconds: 100_000_000)
        }
        XCTAssertTrue(engine.isModelLoaded, "model never finished loading")

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

        let muscle = try XCTUnwrap(engine.lastMuscleResult,
                                   "no muscle output after \(SavitzkyGolayFilter.windowSize) submissions")
        print("ORCH-METRIC activations=\(muscle.activations.count) muscle_ts=\(muscle.timestamp)")

        // --- Rendering diagnostics ------------------------------------
        // MuscleOverlay's path pass draws a capsule for EVERY muscle whose
        // activation clears 0.08. If most of the 520 clear it, the result is a
        // thicket of capsules rather than a readable figure.
        let threshold: Float = 0.08
        let acts = muscle.activations.values.map { Float($0) }
        let over = acts.filter { $0 >= threshold }.count
        let sorted = acts.sorted()
        print("RENDER-METRIC muscles=\(acts.count) over_threshold=\(over) "
            + "min=\(sorted.first ?? 0) median=\(sorted[sorted.count/2]) max=\(sorted.last ?? 0)")

        // Muscle capsules are drawn from world-space endpoints produced by the
        // OpenSim skeleton's FK, while the joint/bone context is drawn from the
        // MHR marker positions. If those two occupy different regions of space
        // the muscles appear detached from the body.
        var pathPts: [SIMD3<Float>] = []
        for (_, p) in muscle.paths { pathPts.append(p.start); pathPts.append(p.end) }
        let jointPts = OfflineMuscleChainFixture.markers.map { $0.2 }
        func bounds(_ ps: [SIMD3<Float>]) -> String {
            guard var lo = ps.first, var hi = ps.first else { return "empty" }
            for p in ps { lo = simd_min(lo, p); hi = simd_max(hi, p) }
            return "min(\(lo.x.rounded3),\(lo.y.rounded3),\(lo.z.rounded3)) max(\(hi.x.rounded3),\(hi.y.rounded3),\(hi.z.rounded3))"
        }
        print("RENDER-METRIC path_count=\(muscle.paths.count) path_bounds=\(bounds(pathPts))")
        print("RENDER-METRIC joint_bounds=\(bounds(jointPts))")

        // The offline runner files results by timestamp; the centred window
        // means this must equal the MIDDLE push, not the last one.
        XCTAssertEqual(muscle.timestamp, 0.0, accuracy: 1e-6,
                       "muscle timestamp is not the centre of the window — timestamp routing in OfflineSessionRunner would misfile or discard it")

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
        XCTAssertEqual(engine.droppedFrameCount, 0,
                       "reset and resubmission must preserve offline backpressure")
    }
}

/// Shared so `OfflineOrchestrationTests` and `OfflineMuscleChainTests` cannot
/// drift apart on the pose they exercise.
enum OfflineMuscleChainFixture {
    static let markers: [(String, String, SIMD3<Float>)] = [
        ("hips_joint", "PELVIS", SIMD3<Float>(0.000000, 0.923987, 0.000000)),
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
