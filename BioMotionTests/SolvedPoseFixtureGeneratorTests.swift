import XCTest
import CryptoKit
import UIKit

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

    // MARK: - The video-driven generator (2026-08-14, 20-marker production fixtures)

    /// Why a clip is excluded from the retained run. The first four are
    /// production's own segment breaks, reproduced in production's order; the
    /// fifth is a Core ML throw, which production also treats as a break
    /// because the slot number then jumps past `areContiguous`.
    enum VideoBreakKind: String {
        case fallbackBBox      // E1 OfflineTemporalPolicy.exclusion, OfflineSessionRunner:1113
        case implausibleBody   // E2 MHRRetarget.plausibility,        OfflineSessionRunner:1150
        case noUsableJoints    // E3 joints.contains(\.isTracked),    OfflineSessionRunner:1168
        case decodeDrop        // E4 `try?` decodeFrame hole,         OfflineSessionRunner:1581
        case poseFailed        // Core ML threw; slot jump breaks contiguity
    }

    /// Unbuffered progress, so a multi-hour Core ML + Debug IK run is
    /// observable while it runs. `print` writes to a block-buffered stdout that
    /// only reaches the lane log at process exit, which is exactly when a stall
    /// stops being diagnosable.
    static func progress(_ line: String) {
        fputs("SOLVED-POSE-FIXTURE-VIDEO-PROGRESS \(line)\n", stderr)
        fflush(stderr)
    }

    /// One sampled slot after decode + Core ML + retarget, before IK.
    private struct SampledSlot {
        let index: Int
        let timestamp: TimeInterval
        let jointCoords: [SIMD3<Float>]
        let bodyFrame: BodyFrame
    }

    /// Regenerates `solved_pose_video_{012,015}.txt` through the FULL production
    /// offline path — video decode → Vision person box → SAM3DBodyPose Core ML →
    /// 127 MHR joints → `MHRRetarget` → per-frame IK — rather than from the
    /// Python-cache-derived `GaitClipFixture` the 5-marker generator above reads.
    ///
    /// Registered 2026-08-14 (20-marker production fixtures, fresh adjudication).
    /// Two deviations from the brief are owner-authorised and recorded in the
    /// fixture header rather than silently applied:
    ///
    /// - **DEVIATION A — no model scaling.** `scaleModelWithHeight` is never
    ///   called. The gate battery's `ModelContext` loads the generic
    ///   `FullBody.osim` and never scales, and scaling mutates segment geometry
    ///   (`NimbleBridge.h:136`), which would move muscle path points and
    ///   desynchronise `q` from the `R(q)`/`L_MT(q)` the gates evaluate. The
    ///   `modelSHA256` staleness guard hashes the FILE and provably could not
    ///   detect that mismatch. `estimatedStatureMeters` and
    ///   `segmentScaleMarkers` are still COMPUTED and recorded as
    ///   measured-but-NOT-APPLIED receipts.
    /// - **DEVIATION B — no Vision revision pin.** Production's
    ///   `SAM3DPoseEstimator.makePersonRectangleRequest()` sets only
    ///   `upperBodyOnly = false` and never touches `.revision`; the pinned
    ///   revisions in this tree belong to the camera-registration subsystem the
    ///   pose path never reaches. Reproduced AS-IS, with OS/Xcode environment
    ///   receipts recorded instead.
    ///
    /// Video paths arrive ONLY through the environment, never as a committed
    /// personal path, and reading + hashing them is this method's first action
    /// so the plumbing probe costs seconds rather than a full run.
    func testRegenerateVideoDrivenSolvedPoseFixtures() async throws {
        let env = ProcessInfo.processInfo.environment

        // --- Plumbing probe: the env vars, FIRST. -----------------------------
        var sources: [(clip: String, url: URL, sha: String, bytes: Int)] = []
        for clipId in Self.scoredClips {
            let suffix = clipId.replacingOccurrences(of: "video_", with: "")
            let key = "BIOMOTION_FIXTURE_VIDEO_\(suffix)"
            guard let path = env[key], !path.isEmpty else {
                XCTFail("""
                    missing environment variable \(key).

                    `tools/run_tests.sh` wraps xcodebuild in `/usr/bin/env -i`, so a plain
                    exported shell variable cannot arrive. Set it as an ENVIRONMENT VARIABLE
                    of the runner, in the TEST_RUNNER_ form, BEFORE the command:

                      TEST_RUNNER_\(key)=/absolute/path/to/\(clipId).mov \\
                      /bin/bash -p tools/run_tests.sh subset \\
                        -only-testing:BioMotionTests/SolvedPoseFixtureGeneratorTests/testRegenerateVideoDrivenSolvedPoseFixtures

                    ⚠️ Do NOT append `TEST_RUNNER_...=...` AFTER the lane as an xcodebuild
                    ARGUMENT. Measured 2026-08-14 on Xcode 26.4/17E192: that form is accepted
                    as a BUILD-SETTING override (it appears in "Build settings from command
                    line") and is never forwarded to the test host, so this same failure
                    repeats after a full run. `man xcodebuild` forwards TEST_RUNNER_-prefixed
                    ENVIRONMENT variables of the xcodebuild PROCESS; `run_tests.sh` re-adds
                    exactly the names in RUN_TESTS_FORWARDED_ENV_NAMES through its scrub.

                    Or run `tools/pose_fixture/regenerate_solved_pose_fixtures.sh --video`,
                    which sets all seven variables for you. The source videos are personal
                    footage OUTSIDE this repository and must never be copied into it.
                    """)
                return
            }
            let url = URL(fileURLWithPath: path)
            guard let data = try? Data(contentsOf: url, options: .mappedIfSafe) else {
                XCTFail("\(key) points at \(path), which could not be read")
                return
            }
            Self.progress("hashing \(clipId) bytes=\(data.count)")
            let sha = SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
            print("SOLVED-POSE-FIXTURE-VIDEO-SOURCE clip=\(clipId) bytes=\(data.count) sha256=\(sha)")
            sources.append((clipId, url, sha, data.count))
        }

        // Host-side receipts the Simulator cannot compute for itself. The
        // regeneration script exports them; `unset` is recorded honestly.
        let macosProduct = env["BIOMOTION_FIXTURE_MACOS_PRODUCT"] ?? "unset"
        let macosBuild = env["BIOMOTION_FIXTURE_MACOS_BUILD"] ?? "unset"
        let xcodeVersion = env["BIOMOTION_FIXTURE_XCODE_VERSION"] ?? "unset"
        let modelLockReceipt = env["BIOMOTION_FIXTURE_MODEL_LOCK_SHA256"] ?? "unset"
        let depsLockSHA = env["BIOMOTION_FIXTURE_DEPS_LOCK_SHA256"] ?? "unset"

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

        // ONE estimator across both clips: `loadModelIfNeeded` is idempotent and
        // the 1.3 GiB cold load is paid once.
        let estimator = SAM3DPoseEstimator()
        Self.progress("loading SAM3DBodyPose (cold)")
        let loadStart = CFAbsoluteTimeGetCurrent()
        try await estimator.loadModelIfNeeded()
        let loadMs = (CFAbsoluteTimeGetCurrent() - loadStart) * 1000
        print("SOLVED-POSE-FIXTURE-VIDEO-MODEL cold_load_ms=\(String(format: "%.1f", loadMs))")

        for source in sources {
            let clipId = source.clip

            // --- 1-3. Decode plan, production sampling. ----------------------
            let decoder = FrameSource.VideoDecoder(url: source.url)
            let duration = try await decoder.duration()
            let fps = await decoder.nominalFrameRate()
            let step = 1.0 / fps
            let wanted = min(FrameSource.maxNativeWindowFrames, max(1, Int(FrameSource.analysisWindowSeconds / step)))
            let available = max(1, Int(duration / step))
            let (timestamps, _) = FrameSource.sampleTimestamps(
                duration: duration,
                mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
                nominalFrameRate: fps)
            let adequacyFloor = Int((FrameSource.minimumAnalysisSeconds * fps).rounded(.up))

            // --- 4-6. Core ML, retarget, and the FOUR (+1) segment breaks. ---
            var slots: [SampledSlot] = []
            var breaks: [(index: Int, kind: VideoBreakKind)] = []
            var samMsTotal = 0.0
            var samCalls = 0

            for (i, t) in timestamps.enumerated() {
                // E4 — one undecodable timestamp leaves a HOLE in `index`.
                guard let image = try? await decoder.decodeFrame(at: t) else {
                    breaks.append((i, .decodeDrop))
                    continue
                }
                let samStart = CFAbsoluteTimeGetCurrent()
                let estimate: SAM3DPoseEstimator.Output
                do {
                    estimate = try await estimator.estimate(uiImage: image)
                } catch {
                    breaks.append((i, .poseFailed))
                    continue
                }
                samMsTotal += (CFAbsoluteTimeGetCurrent() - samStart) * 1000
                samCalls += 1
                if samCalls % 10 == 0 || samCalls == 1 {
                    Self.progress("\(clipId) sam frame=\(i)/\(timestamps.count) "
                                  + "mean_ms=\(Int(samMsTotal / Double(samCalls)))")
                }

                // camT OMITTED, matching production's real call site
                // (OfflineSessionRunner:1104-1106); `frame.index` is the DECODER
                // SLOT, never the array position (:1100-1103).
                let bodyFrame = MHRRetarget.makeBodyFrame(jointCoords: estimate.jointCoords,
                                                          timestamp: t,
                                                          frameNumber: i)

                // E1 — whole-image fallback refusal, FIRST, before plausibility.
                if OfflineTemporalPolicy.exclusion(source: .video,
                                                   usedFallbackBBox: estimate.usedFallbackBBox) != nil {
                    breaks.append((i, .fallbackBBox))
                    continue
                }
                // E2 — body-size gate.
                if case .implausible = MHRRetarget.plausibility(jointCoords: estimate.jointCoords) {
                    breaks.append((i, .implausibleBody))
                    continue
                }
                // E3 — no usable joints.
                guard bodyFrame.joints.contains(where: \.isTracked) else {
                    breaks.append((i, .noUsableJoints))
                    continue
                }
                slots.append(SampledSlot(index: i, timestamp: t,
                                         jointCoords: estimate.jointCoords, bodyFrame: bodyFrame))
            }

            // --- Contiguity: the FIRST maximal contiguous surviving run. -----
            var runs: [[SampledSlot]] = []
            for slot in slots {
                if let last = runs.last?.last,
                   OfflineTemporalPolicy.areContiguous(previousFrameNumber: last.index,
                                                       currentFrameNumber: slot.index) {
                    runs[runs.count - 1].append(slot)
                } else {
                    runs.append([slot])
                }
            }
            let retained = runs.first ?? []
            let branch = breaks.isEmpty ? "A" : "B"

            let excludedText = breaks.isEmpty
                ? "[]"
                : "[" + breaks.map { "\($0.index):\($0.kind.rawValue)" }.joined(separator: ",") + "]"
            let survivingText = retained.isEmpty
                ? "[]"
                : "[\(retained[0].index),\(retained[retained.count - 1].index)]"

            print("SOLVED-POSE-FIXTURE-VIDEO-PLAN clip=\(clipId) duration_s=\(String(format: "%.6f", duration)) "
                  + "fps=\(fps) wanted=\(wanted) available=\(available) sampled=\(timestamps.count) "
                  + "survivors=\(slots.count) runs=\(runs.count) retained=\(retained.count) "
                  + "branch=\(branch) adequacy_floor=\(adequacyFloor) excluded=\(excludedText)")

            // --- ADEQUACY FLOOR, both branches. BLOCKED, never scored. -------
            guard retained.count >= adequacyFloor else {
                XCTFail("""
                    BLOCKED: \(clipId) retained \(retained.count) contiguous frames against the \
                    registered adequacy floor ceil(\(FrameSource.minimumAnalysisSeconds) x \(fps)) = \
                    \(adequacyFloor). Branch \(branch), excluded \(excludedText). No fixture was \
                    written: a clip below the floor is BLOCKED, not scored and not silently accepted.
                    """)
                return
            }

            // --- 7. NO SCALING. Measured-but-not-applied receipts only. ------
            let firstUsable = retained[0]
            let stature = Double(MHRRetarget.estimatedStatureMeters(jointCoords: firstUsable.jointCoords))
            let (_, scaleMarkerNames) = MHRRetarget.segmentScaleMarkers(jointCoords: firstUsable.jointCoords)

            // --- 8. Per-frame IK on ONE bridge, sequential, warm-started. ----
            let bridge = NimbleBridge()
            XCTAssertTrue(bridge.loadModel(fromPath: modelPath), "loadModel failed")
            bridge.resetSessionState()
            let dofNames = bridge.dofNames

            var lines: [String] = []
            var submittedMarkers: [String] = []
            var residualsMM: [Double] = []
            var ikMsTotal = 0.0

            for slot in retained {
                var names: [String] = []
                var positions: [NSNumber] = []
                for joint in slot.bodyFrame.joints {
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

                let ikStart = CFAbsoluteTimeGetCurrent()
                let result = try XCTUnwrap(bridge.solveIK(withMarkerPositions: positions,
                                                          markerNames: names),
                                           "IK returned nil on \(clipId) slot \(slot.index)")
                ikMsTotal += (CFAbsoluteTimeGetCurrent() - ikStart) * 1000
                residualsMM.append(result.markerRMSMeters * 1000)
                if residualsMM.count % 10 == 0 || residualsMM.count == 1 {
                    Self.progress("\(clipId) ik frame=\(residualsMM.count)/\(retained.count) "
                                  + "mean_ms=\(Int(ikMsTotal / Double(residualsMM.count))) "
                                  + "rms_mm=\(String(format: "%.2f", result.markerRMSMeters * 1000)) "
                                  + "iters=\(result.iterations)")
                }
                XCTAssertEqual(result.jointAngles.count, dofNames.count)
                var fields: [String] = ["\(slot.index)",
                                        String(format: "%.9f", slot.timestamp)]
                fields.append(contentsOf: result.jointAngles.map {
                    String(format: "%.12f", $0.doubleValue)
                })
                lines.append(fields.joined(separator: " "))
            }

            let sortedResiduals = residualsMM.sorted()
            let median = sortedResiduals.isEmpty ? 0
                : sortedResiduals[sortedResiduals.count / 2]
            let p95 = sortedResiduals.isEmpty ? 0
                : sortedResiduals[min(sortedResiduals.count - 1,
                                      max(0, Int((0.95 * Double(sortedResiduals.count)).rounded(.up)) - 1))]
            let worst = sortedResiduals.last ?? 0

            var text = ""
            text += "# GENERATED by tools/pose_fixture/regenerate_solved_pose_fixtures.sh --video - do not hand-edit.\n"
            text += "# Raw per-frame IK joint angles for one pinned gait clip, driven through the\n"
            text += "# FULL production offline path: video decode -> Vision person box ->\n"
            text += "# SAM3DBodyPose Core ML -> 127 MHR joints -> MHRRetarget -> per-frame IK on\n"
            text += "# one NimbleBridge, sequentially, warm-started frame to frame.\n"
            text += "# The source video is personal footage OUTSIDE this repository; only its\n"
            text += "# SHA-256 and byte size are recorded here.\n"
            text += "# Angles are radians (metres for the pelvis translations), plain decimals.\n"
            text += "# Columns: frame t then one value per DOF, in the `dofnames` order below.\n"
            text += "format \(Self.formatId)\n"
            text += "generator BioMotionTests/SolvedPoseFixtureGeneratorTests/testRegenerateVideoDrivenSolvedPoseFixtures\n"
            text += "commit UNSET-FILLED-BY-REGENERATION-SCRIPT\n"
            text += "model BioMotion/Resources/FullBody.osim\n"
            text += "model_sha256 \(modelSHA)\n"
            text += "clip \(clipId)\n"
            text += "video_sha256 \(source.sha)\n"
            text += "video_bytes \(source.bytes)\n"
            text += "video_duration_s \(String(format: "%.6f", duration))\n"
            text += "nominal_fps \(fps)\n"
            text += "sampling_policy nativeWindow seconds=\(FrameSource.analysisWindowSeconds) fps=\(fps) "
            text += "wanted=\(wanted) available=\(available) count=\(timestamps.count) "
            text += "start=\(String(format: "%.9f", timestamps.first ?? 0)) step=\(String(format: "%.9f", step)) "
            text += "cap=\(FrameSource.maxNativeWindowFrames) centred=true\n"
            text += "sampling_branch branch=\(branch) excluded=\(excludedText) "
            text += "surviving=\(survivingText) retained=\(retained.count) adequacy_floor=\(adequacyFloor)\n"
            text += "fallback_bbox_frames \(breaks.filter { $0.kind == .fallbackBBox }.count)\n"
            text += "decode_drop_frames \(breaks.filter { $0.kind == .decodeDrop }.count)\n"
            text += "scaling_applied false\n"
            text += "stature_measured_m \(String(format: "%.6f", stature))\n"
            text += "segment_scale_markers n=\(scaleMarkerNames.count),names=\(scaleMarkerNames.joined(separator: "|"))\n"
            text += "ik_residual_mm_median \(String(format: "%.6f", median))\n"
            text += "ik_residual_mm_p95 \(String(format: "%.6f", p95))\n"
            text += "ik_residual_mm_max \(String(format: "%.6f", worst))\n"
            text += "model_lock receipt_sha256=\(modelLockReceipt),deps_lock_sha256=\(depsLockSHA)\n"
            text += "macos_product_version \(macosProduct)\n"
            text += "macos_build_version \(macosBuild)\n"
            text += "xcode_version \(xcodeVersion)\n"
            text += "frames \(retained.count)\n"
            text += "sg_taps 9\n"
            text += "markers \(submittedMarkers.joined(separator: " "))\n"
            text += "dofs \(dofNames.count)\n"
            text += "dofnames \(dofNames.joined(separator: " "))\n"
            text += lines.joined(separator: "\n")
            text += "\n"

            // A DISTINCT filename and a DISTINCT stdout prefix: the 5-marker
            // generator writes `solved_pose_<clip>.txt` into this same directory
            // and prints `SOLVED-POSE-FIXTURE clip=`, which the regeneration
            // script parses with `tail -n 1`. Sharing either would let one
            // generator silently overwrite the other's output.
            let url = outputRoot.appendingPathComponent("solved_pose_videodriven_\(clipId).txt")
            try text.write(to: url, atomically: true, encoding: .utf8)
            print("SOLVED-POSE-FIXTURE-VIDEO clip=\(clipId) frames=\(retained.count) "
                  + "dofs=\(dofNames.count) markers=\(submittedMarkers.count) "
                  + "branch=\(branch) sam_ms_per_frame=\(String(format: "%.1f", samCalls > 0 ? samMsTotal / Double(samCalls) : 0)) "
                  + "sam_calls=\(samCalls) "
                  + "ik_ms_per_frame=\(String(format: "%.1f", retained.isEmpty ? 0 : ikMsTotal / Double(retained.count))) "
                  + "ik_rms_mm_median=\(String(format: "%.4f", median)) "
                  + "ik_rms_mm_p95=\(String(format: "%.4f", p95)) "
                  + "ik_rms_mm_max=\(String(format: "%.4f", worst)) "
                  + "stature_m=\(String(format: "%.4f", stature)) scaling_applied=false "
                  + "path=\(url.path)")
        }
    }
}
