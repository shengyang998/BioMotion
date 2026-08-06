import Combine
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
    private var cancellables = Set<AnyCancellable>()

    override func setUp() {
        super.setUp()
        engine = NimbleEngine()
    }

    override func tearDown() {
        cancellables.removeAll()
        engine = nil
        super.tearDown()
    }

    private func bodyFrame(timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        let joints = OfflineMuscleChainFixture.markers.map { arkitId, _, p in
            TrackedJoint(id: arkitId, name: arkitId, worldPosition: p, isTracked: true)
        }
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    /// Mirrors `OfflineSessionRunner.submitAndWait`: subscribe first, submit,
    /// then wait for the engine to publish.
    @MainActor
    private func submitAndWait(_ frame: BodyFrame, timeout: TimeInterval = 10) async -> Bool {
        await withCheckedContinuation { (cont: CheckedContinuation<Bool, Never>) in
            var resumed = false
            var token: AnyCancellable?
            let finish: (Bool) -> Void = { ok in
                guard !resumed else { return }
                resumed = true
                token?.cancel()
                cont.resume(returning: ok)
            }
            token = engine.objectWillChange
                .receive(on: DispatchQueue.main)
                .sink { _ in
                    DispatchQueue.main.asyncAfter(deadline: .now() + 0.03) { finish(true) }
                }
            engine.processFrame(frame)
            DispatchQueue.main.asyncAfter(deadline: .now() + timeout) { finish(false) }
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
        for push in 0..<SavitzkyGolayFilter.windowSize {
            let ts = Double(push - SavitzkyGolayFilter.halfWindow) * dt
            if await submitAndWait(bodyFrame(timestamp: ts, frameNumber: push)) {
                published += 1
            } else {
                timedOut += 1
            }
            print("ORCH-METRIC push=\(push) ts=\(ts) muscle=\(engine.lastMuscleResult != nil) dropped=\(engine.droppedFrameCount)")
        }

        print("ORCH-METRIC published=\(published) timedOut=\(timedOut) dropped=\(engine.droppedFrameCount)")
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
