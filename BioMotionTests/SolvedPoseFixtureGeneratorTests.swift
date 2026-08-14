import XCTest
import CryptoKit

@testable import BioMotion

/// GENERATOR for `BioMotionTests/Fixtures/solved_pose_video_*.txt`.
///
/// Not a gate and not in any gating lane: `tools/test_gate.sh` skips this whole
/// class in the fast lane exactly as it skips E1, because it is a tool that
/// writes a checked-in artifact rather than a test that decides anything. It is
/// driven by `tools/pose_fixture/regenerate_solved_pose_fixtures.sh`.
///
/// # Why the fixture exists
///
/// The length-mode registration (2026-08-13, owner decision 7) traded a live IK
/// re-solve inside the fast lane for a STORED solved-pose trajectory. IK on
/// these clips is ~1.6 s/frame in a Debug simulator, i.e. about 6.4 minutes for
/// the two scored clips, and the fast lane is 1,381 s today. The Savitzky-Golay
/// stage stays INSIDE the gate — only IK moves out — and the disclosed
/// consequence is that the clip-driven gates test the mode layer GIVEN a pose
/// trajectory, not the IK that produced it.
///
/// # What it writes
///
/// The RAW per-frame IK joint angles (all runtime DOFs, skeleton order), plus a
/// provenance header in the same pattern as the OpenSim fixtures: the generating
/// test, the model path and its SHA-256, the DOF count and ordered DOF names,
/// the marker names actually submitted, and the SG taps the fixture is intended
/// for. The gates re-read the live model's DOF count, DOF name order and model
/// SHA-256 and FAIL if they disagree, so a silent model change cannot leave a
/// stale fixture passing.
///
/// # Provenance of the pose
///
/// Each frame is solved by the SAME production call `NimbleEngine.processFrame`
/// makes — `NimbleBridge.solveIK(withMarkerPositions:markerNames:)` on one
/// bridge instance, sequentially, warm-started frame to frame, on the markers
/// `JointMapping.resolve` admits from the clip's `BodyFrame`. It is driven
/// directly rather than through `processFrame` for one stated reason: after SG
/// warm-up `processFrame` publishes the SMOOTHED pose, and the fixture must hold
/// the RAW series so the gates can run the SG stage themselves.
final class SolvedPoseFixtureGeneratorTests: XCTestCase {

    static let formatId = "biomotion-solved-pose-v1"
    static let scoredClips = ["video_012", "video_015"]

    func testRegenerateSolvedPoseFixtures() throws {
        let bundle = Bundle(for: type(of: self))
        let modelPath = try XCTUnwrap(
            bundle.path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "no FullBody.osim in the bundle")
        let modelData = try Data(contentsOf: URL(fileURLWithPath: modelPath))
        let modelSHA = SHA256.hash(data: modelData).map { String(format: "%02x", $0) }.joined()

        let outputRoot = URL(fileURLWithPath: NSTemporaryDirectory())
            .appendingPathComponent("biomotion-solved-pose", isDirectory: true)
        try? FileManager.default.createDirectory(at: outputRoot,
                                                 withIntermediateDirectories: true)

        for clipId in Self.scoredClips {
            let clip = try GaitClipFixture.load(clipId, bundle: bundle)
            let bridge = NimbleBridge()
            XCTAssertTrue(bridge.loadModel(fromPath: modelPath), "loadModel failed")
            bridge.resetSessionState()

            let dofNames = bridge.dofNames
            var lines: [String] = []
            var submittedMarkers: [String] = []

            for frame in clip.frames {
                var names: [String] = []
                var positions: [NSNumber] = []
                for joint in frame.joints {
                    guard joint.isTracked,
                          let markerName = JointMapping.opensimMarkerName(for: joint)
                    else { continue }
                    guard joint.worldPosition.x.isFinite,
                          joint.worldPosition.y.isFinite,
                          joint.worldPosition.z.isFinite else { continue }
                    names.append(markerName)
                    positions.append(NSNumber(value: Double(joint.worldPosition.x)))
                    positions.append(NSNumber(value: Double(joint.worldPosition.y)))
                    positions.append(NSNumber(value: Double(joint.worldPosition.z)))
                }
                if submittedMarkers.isEmpty { submittedMarkers = names }
                XCTAssertEqual(names, submittedMarkers,
                               "the submitted marker set must not change mid-clip")

                let result = try XCTUnwrap(bridge.solveIK(withMarkerPositions: positions,
                                                          markerNames: names),
                                           "IK returned nil on \(clipId) frame \(frame.frameNumber)")
                XCTAssertEqual(result.jointAngles.count, dofNames.count)
                var fields: [String] = ["\(frame.frameNumber)",
                                        String(format: "%.9f", frame.timestamp)]
                fields.append(contentsOf: result.jointAngles.map {
                    String(format: "%.12f", $0.doubleValue)
                })
                lines.append(fields.joined(separator: " "))
            }

            var text = ""
            text += "# GENERATED by tools/pose_fixture/regenerate_solved_pose_fixtures.sh - do not hand-edit.\n"
            text += "# Raw per-frame IK joint angles for one pinned gait clip, solved by the\n"
            text += "# production NimbleBridge IK call on the markers the clip submits, one\n"
            text += "# bridge instance, sequentially, warm-started frame to frame.\n"
            text += "# Angles are radians (metres for the pelvis translations), plain decimals.\n"
            text += "# Columns: frame t then one value per DOF, in the `dofnames` order below.\n"
            text += "format \(Self.formatId)\n"
            text += "generator BioMotionTests/SolvedPoseFixtureGeneratorTests/testRegenerateSolvedPoseFixtures\n"
            text += "commit UNSET-FILLED-BY-REGENERATION-SCRIPT\n"
            text += "model BioMotion/Resources/FullBody.osim\n"
            text += "model_sha256 \(modelSHA)\n"
            text += "clip \(clipId)\n"
            text += "frames \(clip.frames.count)\n"
            text += "sg_taps 9\n"
            text += "markers \(submittedMarkers.joined(separator: " "))\n"
            text += "dofs \(dofNames.count)\n"
            text += "dofnames \(dofNames.joined(separator: " "))\n"
            text += lines.joined(separator: "\n")
            text += "\n"

            let url = outputRoot.appendingPathComponent("solved_pose_\(clipId).txt")
            try text.write(to: url, atomically: true, encoding: .utf8)
            print("SOLVED-POSE-FIXTURE clip=\(clipId) frames=\(clip.frames.count) "
                  + "dofs=\(dofNames.count) markers=\(submittedMarkers.count) path=\(url.path)")
        }
    }
}
