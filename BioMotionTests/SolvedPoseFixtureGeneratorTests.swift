import XCTest
import CryptoKit
import UIKit

@testable import BioMotion

// MARK: - Person-box sidecar (2026-08-14 fifteenth-round amendment)

/// The parsed + verified person-box sidecar the video-driven generator consumes
/// INSTEAD of calling `SAM3DPoseEstimator.detectPersonBBox` live.
///
/// # Why this exists
///
/// `VNDetectHumanRectanglesRequest.perform` THROWS on every frame inside the
/// iOS Simulator — `com.apple.Vision Code=9 "Could not create inference
/// context"`, 12/12 measured 2026-08-14, with the face-rect and body-pose
/// controls throwing their own distinct messages — while decode in that same
/// host is proven healthy. macOS Vision, same request configuration, found a
/// person on 12/12 sampled frames of both clips. So the box is computed by a
/// macOS HOST tool (`tools/pose_fixture/person_box_sidecar.swift`) and consumed
/// here. That is an ENVIRONMENT limitation, not a defect in this generator.
///
/// # What is and is not claimed
///
/// macOS Vision provenance is NOT iOS Vision provenance. The fixture header
/// records `bbox_source macos_vision INTERIM`. Equivalence is claimed for the
/// CONVERSION and the CROP given the numbers, never for the numbers themselves.
///
/// # Why the comparator is a value type with pure methods
///
/// So the generator can prove it REFUSES fabricated bad input before it consumes
/// a single box (`SOLVED-POSE-FIXTURE-SIDECAR-SELFTEST`), and so the
/// convention-refusal half can additionally be pinned in the FAST lane by
/// `PersonBoxTests` — which is the only lane that gates anything, since this
/// whole class is skipped there.
struct SidecarPlan {

    /// A refusal is FAIL-CLOSED and NAMES what disagreed. Any refusal blocks the
    /// round before a box is consumed and before a fixture byte is written.
    struct Refusal: Error {
        let part: String          // "A" identity/conventions, "B" plan, "C" well-formedness
        let field: String
        let firstBadIndex: Int?
        let generatorValue: String
        let sidecarValue: String

        var marker: String {
            "part=\(part) field=\(field) "
            + "first_bad_index=\(firstBadIndex.map(String.init) ?? "-") "
            + "generator_value=\(generatorValue) sidecar_value=\(sidecarValue)"
        }
    }

    struct Frame {
        let i: Int
        let ptsBits: UInt64
        let status: String
        let decodedWidth: Int
        let decodedHeight: Int
        let pixelBytes: Int
        let sourceHash: UInt64
        let lumaMean: Double
        let lumaSD: Double
        /// RAW Vision normalized box: bottom-left origin, Y-up. NO flip has been
        /// applied — the flip lives in exactly one place,
        /// `SAM3DPoseEstimator.personBoxPixels`.
        let box: CGRect?
        let confidence: Double?
        let observations: Int?
    }

    /// Everything the GENERATOR independently knows and the sidecar must agree
    /// with. Nothing here is read out of the sidecar.
    struct Expectation {
        let clip: String
        let videoSHA: String
        let videoBytes: Int
        let toolSourceSHA: String
        let timestamps: [TimeInterval]
        let duration: Double
        let fps: Double
        let step: Double
        let start: Double
        let wanted: Int
        let available: Int
    }

    static let schemaId = "biomotion.person_box_sidecar.v1"
    static let bboxSpace = "vision_normalized_bottom_left_origin_y_up"
    static let sourceHashAlgorithm =
        "fnv1a64_over_premultipliedLast_deviceRGB_8bpc_rowBytes_eq_w_times_4"
    /// The bare FNV-1a offset basis, rejected as a NO-PIXELS sentinel on BOTH
    /// sides — it closes `SAM3DPoseEstimator.sourcePixels`' empty-array
    /// degenerate case, which would otherwise hash to this on every frame.
    static let fnvOffsetBasis: UInt64 = 0xcbf29ce484222325

    let schema: String
    let clip: String
    let toolSourceSHA: String
    let swiftVersion: String
    let logPath: String
    let macosProduct: String
    let macosBuild: String
    let xcodeVersion: String
    let videoSHA: String
    let videoBytes: Int
    let naturalWidth: Int
    let naturalHeight: Int
    let requestClass: String
    let upperBodyOnly: Bool
    let revisionPinned: Bool
    let handlerOrientation: String
    let revisionUsed: Int
    let appliesPreferredTrackTransform: Bool
    let toleranceBefore: String
    let toleranceAfter: String
    let preferredTimescale: Int
    let mode: String
    let seconds: Double
    let capMaxNativeWindowFrames: Int
    let durationBits: UInt64
    let fpsBits: UInt64
    let stepBits: UInt64
    let startBits: UInt64
    let wanted: Int
    let available: Int
    let count: Int
    let bboxSpace: String
    let sourceHashAlgorithm: String
    let frames: [Frame]

    /// The complete, reachable set of per-frame outcomes. An UNKNOWN status
    /// string is schema drift and BLOCKS; it is never read as "not found".
    /// There is deliberately NO `decode_failed` status (a host decode failure
    /// makes the host emit no sidecar at all) and NO "record missing" cause
    /// (a hole in the sidecar is a part-B block, not an E1 break).
    static let knownStatuses: Set<String> = ["found", "no_observation", "perform_threw"]

    // MARK: Parse

    private static func fail(_ part: String, _ field: String, _ index: Int?,
                            _ expected: String, _ got: String) -> Refusal {
        Refusal(part: part, field: field, firstBadIndex: index,
                generatorValue: expected, sidecarValue: got)
    }

    static func parse(_ data: Data) throws -> SidecarPlan {
        guard let any = try? JSONSerialization.jsonObject(with: data),
              let root = any as? [String: Any] else {
            throw fail("A", "json", nil, "a JSON object", "unparseable")
        }
        func obj(_ key: String) throws -> [String: Any] {
            guard let v = root[key] as? [String: Any] else {
                throw fail("A", key, nil, "an object", "missing or not an object")
            }
            return v
        }
        func str(_ d: [String: Any], _ key: String, _ where_: String) throws -> String {
            guard let v = d[key] as? String else {
                throw fail("A", "\(where_).\(key)", nil, "a string", "missing or not a string")
            }
            return v
        }
        func int(_ d: [String: Any], _ key: String, _ where_: String) throws -> Int {
            guard let v = d[key] as? NSNumber else {
                throw fail("A", "\(where_).\(key)", nil, "an integer", "missing or not a number")
            }
            return v.intValue
        }
        func dbl(_ d: [String: Any], _ key: String, _ where_: String) throws -> Double {
            guard let v = d[key] as? NSNumber else {
                throw fail("A", "\(where_).\(key)", nil, "a number", "missing or not a number")
            }
            return v.doubleValue
        }
        func bool(_ d: [String: Any], _ key: String, _ where_: String) throws -> Bool {
            guard let v = d[key] as? NSNumber else {
                throw fail("A", "\(where_).\(key)", nil, "a bool", "missing or not a bool")
            }
            return v.boolValue
        }
        func bits(_ d: [String: Any], _ key: String, _ where_: String,
                  _ part: String, _ index: Int?) throws -> UInt64 {
            guard let s = d[key] as? String, let v = UInt64(s) else {
                throw fail(part, "\(where_).\(key)", index,
                           "a UInt64 bit pattern as a decimal string",
                           "missing or unparseable")
            }
            return v
        }

        let tool = try obj("tool")
        let environment = try obj("environment")
        let video = try obj("video")
        let request = try obj("request")
        let decode = try obj("decode")
        let sampling = try obj("sampling_plan")

        guard let rawFrames = root["frames"] as? [[String: Any]] else {
            throw fail("B", "frames", nil, "an array of objects", "missing or not an array")
        }
        var parsedFrames: [Frame] = []
        parsedFrames.reserveCapacity(rawFrames.count)
        for (n, f) in rawFrames.enumerated() {
            let status = try str(f, "status", "frames[\(n)]")
            guard knownStatuses.contains(status) else {
                throw fail("A", "frames[].status", n,
                           knownStatuses.sorted().joined(separator: "|"), status)
            }
            var box: CGRect?
            var confidence: Double?
            var observations: Int?
            if status == "found" {
                let x = Double(bitPattern: try bits(f, "x_bits", "frames[\(n)]", "C", n))
                let y = Double(bitPattern: try bits(f, "y_bits", "frames[\(n)]", "C", n))
                let w = Double(bitPattern: try bits(f, "w_bits", "frames[\(n)]", "C", n))
                let h = Double(bitPattern: try bits(f, "h_bits", "frames[\(n)]", "C", n))
                box = CGRect(x: x, y: y, width: w, height: h)
                confidence = try dbl(f, "confidence", "frames[\(n)]")
                observations = try int(f, "observations", "frames[\(n)]")
            }
            parsedFrames.append(Frame(
                i: try int(f, "i", "frames[\(n)]"),
                ptsBits: try bits(f, "pts_bits", "frames[\(n)]", "B", n),
                status: status,
                decodedWidth: try int(f, "decoded_width", "frames[\(n)]"),
                decodedHeight: try int(f, "decoded_height", "frames[\(n)]"),
                pixelBytes: try int(f, "pixel_bytes", "frames[\(n)]"),
                sourceHash: try bits(f, "source_hash", "frames[\(n)]", "C", n),
                lumaMean: try dbl(f, "luma_mean", "frames[\(n)]"),
                lumaSD: try dbl(f, "luma_sd", "frames[\(n)]"),
                box: box, confidence: confidence, observations: observations))
        }

        return SidecarPlan(
            schema: try str(root, "schema", "root"),
            clip: try str(root, "clip", "root"),
            toolSourceSHA: try str(tool, "source_sha256", "tool"),
            swiftVersion: try str(tool, "swift_version", "tool"),
            logPath: try str(tool, "log_path", "tool"),
            macosProduct: try str(environment, "macos_product_version", "environment"),
            macosBuild: try str(environment, "macos_build_version", "environment"),
            xcodeVersion: try str(environment, "xcode_version", "environment"),
            videoSHA: try str(video, "sha256", "video"),
            videoBytes: try int(video, "bytes", "video"),
            naturalWidth: try int(video, "natural_width", "video"),
            naturalHeight: try int(video, "natural_height", "video"),
            requestClass: try str(request, "class", "request"),
            upperBodyOnly: try bool(request, "upper_body_only", "request"),
            revisionPinned: try bool(request, "revision_pinned", "request"),
            handlerOrientation: try str(request, "handler_orientation", "request"),
            revisionUsed: try int(request, "revision_used", "request"),
            appliesPreferredTrackTransform:
                try bool(decode, "applies_preferred_track_transform", "decode"),
            toleranceBefore: try str(decode, "requested_time_tolerance_before", "decode"),
            toleranceAfter: try str(decode, "requested_time_tolerance_after", "decode"),
            preferredTimescale: try int(decode, "preferred_timescale", "decode"),
            mode: try str(sampling, "mode", "sampling_plan"),
            seconds: try dbl(sampling, "seconds", "sampling_plan"),
            capMaxNativeWindowFrames:
                try int(sampling, "cap_max_native_window_frames", "sampling_plan"),
            durationBits: try bits(sampling, "duration_seconds_bits", "sampling_plan", "B", nil),
            fpsBits: try bits(sampling, "fps_bits", "sampling_plan", "B", nil),
            stepBits: try bits(sampling, "step_bits", "sampling_plan", "B", nil),
            startBits: try bits(sampling, "start_bits", "sampling_plan", "B", nil),
            wanted: try int(sampling, "wanted", "sampling_plan"),
            available: try int(sampling, "available", "sampling_plan"),
            count: try int(sampling, "count", "sampling_plan"),
            bboxSpace: try str(root, "bbox_space", "root"),
            sourceHashAlgorithm: try str(root, "source_hash_algorithm", "root"),
            frames: parsedFrames)
    }

    // MARK: Part A — identity, build binding, CONVENTION conformance

    /// EVERY convention field the schema carries is a GATE. A sidecar produced
    /// with `upperBodyOnly = true`, a non-`.up` handler orientation, a pinned
    /// revision, a different decode convention, a different sampling mode or a
    /// different bbox space is REFUSED with the offending field NAMED — it is
    /// never silently consumed.
    func verifyConventions() -> Refusal? {
        func eq<T: Equatable>(_ field: String, _ expected: T, _ got: T) -> Refusal? {
            got == expected ? nil
                : Self.fail("A", field, nil, "\(expected)", "\(got)")
        }
        if let r = eq("schema", Self.schemaId, schema) { return r }
        if let r = eq("request.class", "VNDetectHumanRectanglesRequest", requestClass) { return r }
        if let r = eq("request.upper_body_only", false, upperBodyOnly) { return r }
        if let r = eq("request.revision_pinned", false, revisionPinned) { return r }
        if let r = eq("request.handler_orientation", "up", handlerOrientation) { return r }
        if let r = eq("decode.applies_preferred_track_transform", true,
                      appliesPreferredTrackTransform) { return r }
        if let r = eq("decode.requested_time_tolerance_before", "zero", toleranceBefore) { return r }
        if let r = eq("decode.requested_time_tolerance_after", "zero", toleranceAfter) { return r }
        if let r = eq("decode.preferred_timescale", 600, preferredTimescale) { return r }
        if let r = eq("sampling_plan.mode", "nativeWindow", mode) { return r }
        if let r = eq("sampling_plan.seconds", FrameSource.analysisWindowSeconds, seconds) { return r }
        if let r = eq("sampling_plan.cap_max_native_window_frames",
                      FrameSource.maxNativeWindowFrames, capMaxNativeWindowFrames) { return r }
        if let r = eq("bbox_space", Self.bboxSpace, bboxSpace) { return r }
        if let r = eq("source_hash_algorithm", Self.sourceHashAlgorithm,
                      sourceHashAlgorithm) { return r }
        return nil
    }

    /// Identity + BUILD BINDING. The generator recomputes
    /// `sha256(tools/pose_fixture/person_box_sidecar.swift)` from the tree it
    /// runs in and refuses on mismatch, so a sidecar from any other tool version
    /// is dead on arrival. The video hash/byte count are what the GENERATOR
    /// itself read, never what the host claims about itself.
    func verifyIdentity(_ e: Expectation) -> Refusal? {
        if clip != e.clip {
            return Self.fail("A", "clip", nil, e.clip, clip)
        }
        if videoSHA != e.videoSHA {
            return Self.fail("A", "video.sha256", nil, e.videoSHA, videoSHA)
        }
        if videoBytes != e.videoBytes {
            return Self.fail("A", "video.bytes", nil, "\(e.videoBytes)", "\(videoBytes)")
        }
        if toolSourceSHA != e.toolSourceSHA {
            return Self.fail("A", "tool.source_sha256", nil, e.toolSourceSHA, toolSourceSHA)
        }
        return nil
    }

    // MARK: Part B — plan equality, BIT-EXACT, tolerance ZERO

    /// ONE ULP of disagreement BLOCKS the round, with BOTH bit patterns
    /// recorded. No tolerance is invented here; if this ever fires, a SUCCESSOR
    /// round may register one derived from the measured delta — never before.
    /// The comparison is on exact integers, so it is immune to JSON decimal
    /// formatting and needs no reasoning about `Double ==` semantics.
    func verifyPlan(_ e: Expectation) -> Refusal? {
        func eqBits(_ field: String, _ expected: Double, _ got: UInt64) -> Refusal? {
            expected.bitPattern == got ? nil
                : Self.fail("B", field, nil, "\(expected.bitPattern)", "\(got)")
        }
        if let r = eqBits("sampling_plan.duration_seconds_bits", e.duration, durationBits) { return r }
        if let r = eqBits("sampling_plan.fps_bits", e.fps, fpsBits) { return r }
        if let r = eqBits("sampling_plan.step_bits", e.step, stepBits) { return r }
        if let r = eqBits("sampling_plan.start_bits", e.start, startBits) { return r }
        if wanted != e.wanted {
            return Self.fail("B", "sampling_plan.wanted", nil, "\(e.wanted)", "\(wanted)")
        }
        if available != e.available {
            return Self.fail("B", "sampling_plan.available", nil, "\(e.available)", "\(available)")
        }
        if count != e.timestamps.count {
            return Self.fail("B", "sampling_plan.count", nil,
                             "\(e.timestamps.count)", "\(count)")
        }
        if frames.count != e.timestamps.count {
            return Self.fail("B", "frames.count", nil, "\(e.timestamps.count)", "\(frames.count)")
        }
        for (i, t) in e.timestamps.enumerated() {
            if frames[i].i != i {
                return Self.fail("B", "frames[].i", i, "\(i)", "\(frames[i].i)")
            }
            if frames[i].ptsBits != t.bitPattern {
                return Self.fail("B", "frames[].pts_bits", i,
                                 "\(t.bitPattern)", "\(frames[i].ptsBits)")
            }
        }
        return nil
    }

    // MARK: Part C — per-frame well-formedness (the image-independent half)

    /// Structural checks on values Vision DEFINES to be in range — they cannot
    /// spuriously fire. NO AREA CEILING is imposed: a legitimate box can be
    /// large, so a full-frame box is made VISIBLE by the box-trajectory summary
    /// rather than refused.
    func verifyFrameShapes() -> Refusal? {
        for f in frames {
            if f.pixelBytes != f.decodedWidth * f.decodedHeight * 4 {
                return Self.fail("C", "frames[].pixel_bytes", f.i,
                                 "\(f.decodedWidth * f.decodedHeight * 4)", "\(f.pixelBytes)")
            }
            if f.sourceHash == Self.fnvOffsetBasis {
                return Self.fail("C", "frames[].source_hash", f.i,
                                 "a hash over real pixels",
                                 "the bare FNV offset basis (no pixels were hashed)")
            }
            if f.status == "found" {
                guard let n = f.observations, n >= 1 else {
                    return Self.fail("C", "frames[].observations", f.i, ">= 1",
                                     f.observations.map(String.init) ?? "missing")
                }
                guard let c = f.confidence, c > 0, c <= 1 else {
                    return Self.fail("C", "frames[].confidence", f.i, "in (0, 1]",
                                     f.confidence.map { "\($0)" } ?? "missing")
                }
            }
        }
        return nil
    }

    /// The whole admission, in the registered order. Pure: it reads only the
    /// bytes and the generator's own `Expectation`, which is what makes the
    /// in-run self-test over FABRICATED sidecars meaningful.
    static func admit(_ data: Data, expectation: Expectation) -> Refusal? {
        let plan: SidecarPlan
        do { plan = try parse(data) } catch let r as Refusal { return r } catch {
            return fail("A", "json", nil, "a parseable sidecar", "\(error)")
        }
        if let r = plan.verifyConventions() { return r }
        if let r = plan.verifyIdentity(expectation) { return r }
        if let r = plan.verifyPlan(expectation) { return r }
        if let r = plan.verifyFrameShapes() { return r }
        return nil
    }
}

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

    // MARK: - Sidecar comparator: its own negative coverage

    /// A minimal well-formed sidecar, plus a mutation hook, so the comparator
    /// can be shown to REFUSE fabricated bad input. Nothing here touches a real
    /// clip: the point is that the refusal path exists and fails closed.
    private static func fabricatedSidecar(
        _ e: SidecarPlan.Expectation,
        mutate: (inout [String: Any]) -> Void = { _ in }
    ) -> Data {
        var frames: [[String: Any]] = []
        for (i, t) in e.timestamps.enumerated() {
            frames.append([
                "i": i,
                "pts_bits": String(t.bitPattern),
                "pts": String(format: "%.9f", t),
                "status": "found",
                "decoded_width": 576, "decoded_height": 1024,
                "pixel_bytes": 576 * 1024 * 4,
                "source_hash": String(UInt64(1_234_567) &+ UInt64(i)),
                "luma_mean": 100.0, "luma_sd": 40.0,
                "x_bits": String((0.3).bitPattern), "y_bits": String((0.2).bitPattern),
                "w_bits": String((0.33).bitPattern), "h_bits": String((0.41).bitPattern),
                "x": 0.3, "y": 0.2, "w": 0.33, "h": 0.41,
                "confidence": 0.7, "observations": 1,
            ])
        }
        var root: [String: Any] = [
            "schema": SidecarPlan.schemaId,
            "clip": e.clip,
            "tool": ["name": "tools/pose_fixture/person_box_sidecar",
                     "source_sha256": e.toolSourceSHA,
                     "swift_version": "Apple Swift version X",
                     "log_path": "/dev/null"],
            "environment": ["macos_product_version": "0.0",
                            "macos_build_version": "0",
                            "xcode_version": "Xcode X Build version Y"],
            "video": ["sha256": e.videoSHA, "bytes": e.videoBytes,
                      "natural_width": 576, "natural_height": 1024],
            "request": ["class": "VNDetectHumanRectanglesRequest",
                        "upper_body_only": false, "revision_pinned": false,
                        "handler_orientation": "up", "revision_used": 2,
                        "supported_revisions": [2]],
            "decode": ["applies_preferred_track_transform": true,
                       "requested_time_tolerance_before": "zero",
                       "requested_time_tolerance_after": "zero",
                       "preferred_timescale": 600,
                       "maximum_size_w": 1024, "maximum_size_h": 1024],
            "sampling_plan": ["mode": "nativeWindow",
                              "seconds": FrameSource.analysisWindowSeconds,
                              "cap_max_native_window_frames": FrameSource.maxNativeWindowFrames,
                              "duration_seconds_bits": String(e.duration.bitPattern),
                              "fps_bits": String(e.fps.bitPattern),
                              "step_bits": String(e.step.bitPattern),
                              "start_bits": String(e.start.bitPattern),
                              "wanted": e.wanted, "available": e.available,
                              "count": e.timestamps.count],
            "bbox_space": SidecarPlan.bboxSpace,
            "source_hash_algorithm": SidecarPlan.sourceHashAlgorithm,
            "frames": frames,
            "summary": ["count": frames.count, "found": frames.count,
                        "no_observation": 0, "perform_threw": 0],
        ]
        mutate(&root)
        return (try? JSONSerialization.data(withJSONObject: root)) ?? Data()
    }

    /// The same fabricator, reachable from `PersonBoxTests` so the FAST lane can
    /// pin the convention-refusal half. This class is whole-class-skipped there,
    /// so without this the refusal path would have no gating coverage at all.
    static func fabricatedSidecarForPinning(_ e: SidecarPlan.Expectation,
                                            _ mutate: (inout [String: Any]) -> Void) -> Data {
        fabricatedSidecar(e, mutate: mutate)
    }

    /// TEN fabricated sidecars that MUST be refused, plus ONE well-formed
    /// sidecar that MUST be accepted — a self-test that refuses everything is
    /// otherwise indistinguishable from a working one.
    static func runSidecarComparatorSelfTest() -> Bool {
        let fps = 30.0
        let step = 1.0 / fps
        let start = 3.15
        let timestamps = (0..<3).map { start + Double($0) * step }
        let e = SidecarPlan.Expectation(
            clip: "video_012", videoSHA: String(repeating: "a", count: 64), videoBytes: 4242,
            toolSourceSHA: String(repeating: "b", count: 64), timestamps: timestamps,
            duration: 10.0, fps: fps, step: step, start: start, wanted: 120, available: 300)

        func mutateFrame(_ root: inout [String: Any], _ index: Int,
                         _ apply: (inout [String: Any]) -> Void) {
            guard var fs = root["frames"] as? [[String: Any]], index < fs.count else { return }
            var f = fs[index]
            apply(&f)
            fs[index] = f
            root["frames"] = fs
        }

        let cases: [(name: String, data: Data, mustRefuse: Bool)] = [
            ("well_formed", fabricatedSidecar(e), false),
            ("count_short_by_one", fabricatedSidecar(e) { root in
                if var fs = root["frames"] as? [[String: Any]] {
                    fs.removeLast(); root["frames"] = fs
                }
            }, true),
            ("count_long_by_one", fabricatedSidecar(e) { root in
                if var fs = root["frames"] as? [[String: Any]], let last = fs.last {
                    var extra = last; extra["i"] = fs.count; fs.append(extra); root["frames"] = fs
                }
            }, true),
            ("pts_bits_off_by_one_ulp", fabricatedSidecar(e) { root in
                mutateFrame(&root, 1) { f in
                    f["pts_bits"] = String(timestamps[1].bitPattern &+ 1)
                }
            }, true),
            ("duplicated_out_of_order_index", fabricatedSidecar(e) { root in
                mutateFrame(&root, 1) { f in f["i"] = 0 }
            }, true),
            ("missing_pts_bits_key", fabricatedSidecar(e) { root in
                mutateFrame(&root, 2) { f in f.removeValue(forKey: "pts_bits") }
            }, true),
            ("fps_bits_off_by_one_ulp", fabricatedSidecar(e) { root in
                if var s = root["sampling_plan"] as? [String: Any] {
                    s["fps_bits"] = String(fps.bitPattern &+ 1); root["sampling_plan"] = s
                }
            }, true),
            ("video_sha256_one_hex_char", fabricatedSidecar(e) { root in
                if var v = root["video"] as? [String: Any] {
                    v["sha256"] = String(repeating: "a", count: 63) + "c"; root["video"] = v
                }
            }, true),
            ("unknown_status", fabricatedSidecar(e) { root in
                mutateFrame(&root, 0) { f in f["status"] = "definitely_not_a_registered_status" }
            }, true),
            ("changed_schema", fabricatedSidecar(e) { root in
                root["schema"] = "biomotion.person_box_sidecar.v2"
            }, true),
            ("upper_body_only_true", fabricatedSidecar(e) { root in
                if var r = root["request"] as? [String: Any] {
                    r["upper_body_only"] = true; root["request"] = r
                }
            }, true),
        ]

        var ok = true
        for c in cases {
            let refusal = SidecarPlan.admit(c.data, expectation: e)
            let refused = refusal != nil
            print("SOLVED-POSE-FIXTURE-SIDECAR-SELFTEST case=\(c.name) refused=\(refused)"
                  + (refusal.map { " \($0.marker)" } ?? ""))
            if refused != c.mustRefuse {
                XCTFail("BLOCKED: the sidecar comparator's self-test case \(c.name) "
                        + "expected refused=\(c.mustRefuse) and got refused=\(refused). "
                        + "A comparator that cannot reject fabricated bad input may not "
                        + "consume a real sidecar.")
                ok = false
            }
        }
        return ok
    }

    /// Pins the CONVENTION, not a re-measurement of Vision: Vision's normalized
    /// box is bottom-left-origin / Y-up and `personBoxPixels` must flip it into
    /// the top-left-origin / Y-down pixel space everything downstream uses.
    ///
    /// Numbers are the 3-decimal box printed for frame 0 of `video_012` by the
    /// macOS probe (`/tmp/visionprobe/run.log`), against the decoded size the
    /// Simulator probe reports for the same clip (576x1024). A wrong-signed flip
    /// yields `y1 = 243.712` instead of `356.352` — 112.6 px away, so this pin
    /// discriminates loudly rather than to the last digit.
    static func runPersonBoxFlipPin() -> Bool {
        let nb = CGRect(x: 0.303, y: 0.238, width: 0.334, height: 0.414)
        let size = CGSize(width: 576, height: 1024)
        guard let rect = SAM3DPoseEstimator.personBoxPixels(
            visionNormalizedBottomLeftOriginYUp: nb, imageSize: size) else {
            XCTFail("BLOCKED: the flip pin's box was refused as degenerate")
            return false
        }
        let expected = CGRect(x: 174.528, y: 356.352, width: 192.384, height: 423.936)
        print("SOLVED-POSE-FIXTURE-SIDECAR-FLIP-PIN "
              + String(format: "x=%.6f y=%.6f w=%.6f h=%.6f", rect.minX, rect.minY,
                       rect.width, rect.height))
        var ok = true
        for (name, got, want) in [("x", rect.minX, expected.minX),
                                  ("y", rect.minY, expected.minY),
                                  ("w", rect.width, expected.width),
                                  ("h", rect.height, expected.height)] {
            if abs(got - want) > 1e-6 {
                XCTFail("BLOCKED: the person-box flip pin's \(name) reads \(got) against \(want)")
                ok = false
            }
        }
        return ok
    }

    /// FIELD-CLASS SANITIZATION, stated once and implemented identically by the
    /// host tool.
    ///
    /// VERSION / RECEIPT class — `swift_version`, `xcode_version`,
    /// `macos_product_version`, `macos_build_version`, lock SHA-256s, the Vision
    /// revision: newlines become SPACES and `/` is **PERMITTED**, because a
    /// version string legitimately contains one. (The earlier draft's blanket
    /// "refuse any provenance string containing `/`" is withdrawn: it was false
    /// against this generator's own committed `xcode_version` line and would
    /// have BLOCKED the round on itself after a multi-hour host run.)
    ///
    /// PATH-LIKE class — the fixture's `person_box_log`, the sidecar basename,
    /// the tool name: `/` is REFUSED, so a directory component can never reach a
    /// committed fixture. That refusal is at each call site, not here.
    static func receiptSanitized(_ raw: String) -> String {
        raw.replacingOccurrences(of: "\r\n", with: " ")
            .replacingOccurrences(of: "\n", with: " ")
            .replacingOccurrences(of: "\r", with: " ")
            .trimmingCharacters(in: .whitespacesAndNewlines)
    }

    /// Rec.709 luma over EVERY pixel of the same `premultipliedLast` deviceRGB
    /// buffer the source hash is taken from — the identical formula the host
    /// tool implements. Receipt only: it is emitted as a DELTA when content
    /// binding reads UNPROVEN, and never gates anything.
    static func luminanceStats(_ buf: [UInt8]) -> (mean: Double, sd: Double) {
        guard buf.count >= 4 else { return (0, 0) }
        var sum = 0.0, sumSq = 0.0
        var n = 0
        var i = 0
        while i + 3 < buf.count {
            let l = 0.2126 * Double(buf[i]) + 0.7152 * Double(buf[i + 1])
                + 0.0722 * Double(buf[i + 2])
            sum += l
            sumSq += l * l
            n += 1
            i += 4
        }
        guard n > 0 else { return (0, 0) }
        let mean = sum / Double(n)
        return (mean, max(0, sumSq / Double(n) - mean * mean).squareRoot())
    }

    /// One sampled slot after decode + Core ML + retarget, before IK.
    private struct SampledSlot {
        let index: Int
        let timestamp: TimeInterval
        let jointCoords: [SIMD3<Float>]
        let bodyFrame: BodyFrame
    }

    /// Regenerates `solved_pose_video_{012,015}.txt` through the production
    /// offline path — video decode → person box via macOS-host sidecar →
    /// SAM3DBodyPose Core ML → 127 MHR joints → `MHRRetarget` → per-frame IK —
    /// rather than from the Python-cache-derived `GaitClipFixture` the 5-marker
    /// generator above reads.
    ///
    /// Registered 2026-08-14 (20-marker production fixtures, fresh adjudication).
    /// Three deviations from the brief are owner-authorised and recorded in the
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
    /// - **DEVIATION C — the person box comes from a macOS-host SIDECAR**
    ///   (registered 2026-08-14, fifteenth round). The iOS Simulator has no
    ///   Vision ML inference backend: `VNDetectHumanRectanglesRequest.perform`
    ///   THREW on 12/12 sampled frames (`com.apple.Vision Code=9 "Could not
    ///   create inference context"`), with face-rect and body-pose controls
    ///   throwing their own distinct messages, while decode in the same host is
    ///   healthy. `tools/pose_fixture/person_box_sidecar.swift` computes the box
    ///   per sampled frame on macOS with the SAME request configuration and the
    ///   SAME decode conventions; this generator recomputes the sampling plan
    ///   through the real `FrameSource.sampleTimestamps` and refuses on ONE ULP
    ///   of disagreement before consuming a box. SAM, `MHRRetarget`, IK and
    ///   E2/E3/E4 are untouched. `bbox_source macos_vision INTERIM` is recorded;
    ///   macOS Vision provenance is NOT iOS Vision provenance and nothing
    ///   transitioned on these fixtures is quotable as device-grade.
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
                    which sets all nine variables for you. The source videos are personal
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

        // --- Person-box sidecars: one per clip, produced by the macOS host tool.
        // Read AFTER the video probe so the doc comment above stays literally
        // true, and BEFORE anything expensive for the same reason it exists.
        var sidecarBytes: [String: (url: URL, data: Data, sha: String)] = [:]
        for clipId in Self.scoredClips {
            let suffix = clipId.replacingOccurrences(of: "video_", with: "")
            let key = "BIOMOTION_FIXTURE_BOX_SIDECAR_\(suffix)"
            guard let path = env[key], !path.isEmpty else {
                XCTFail("""
                    missing environment variable \(key).

                    The iOS Simulator has NO Vision ML inference backend: measured 2026-08-14,
                    VNDetectHumanRectanglesRequest.perform THREW on 12/12 sampled frames
                    (com.apple.Vision Code=9 "Could not create inference context"), with the
                    face-rect and body-pose controls throwing their own distinct messages,
                    while decode in the same host is healthy. The person box is therefore
                    computed by the macOS HOST tool tools/pose_fixture/person_box_sidecar.swift
                    and consumed here. Run

                      /bin/bash -p tools/pose_fixture/regenerate_solved_pose_fixtures.sh --video

                    which builds the host tool, runs it on both clips, and exports all nine
                    TEST_RUNNER_ variables. Sidecars are derivatives of personal footage and
                    live OUTSIDE this repository, exactly like the videos.
                    """)
                return
            }
            let url = URL(fileURLWithPath: path)
            // The FILENAME is derived by the host tool and re-asserted here: an
            // operator-chosen path must not be able to smuggle a differently
            // named artefact past the guards that keep it out of the tree.
            let expectedName = "person_box_sidecar_\(clipId).json"
            guard url.lastPathComponent == expectedName else {
                XCTFail("BLOCKED: \(key) names \(url.lastPathComponent); the sidecar basename "
                        + "must be exactly \(expectedName)")
                return
            }
            guard let data = try? Data(contentsOf: url) else {
                XCTFail("\(key) points at \(path), which could not be read")
                return
            }
            let sha = SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
            print("SOLVED-POSE-FIXTURE-SIDECAR-SOURCE clip=\(clipId) bytes=\(data.count) sha256=\(sha)")
            sidecarBytes[clipId] = (url, data, sha)
        }

        // --- BUILD BINDING: the host tool's source SHA-256, recomputed from the
        // tree this test runs in. A sidecar written by any other version of the
        // tool is dead on arrival. Repo root comes from the house `#filePath`
        // pattern, so no personal absolute path enters committed source and no
        // third environment name is needed.
        let repoRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent().deletingLastPathComponent()
        let hostToolURL = repoRoot
            .appendingPathComponent("tools/pose_fixture/person_box_sidecar.swift")
        guard let hostToolData = try? Data(contentsOf: hostToolURL) else {
            XCTFail("BLOCKED: could not read the host tool source at "
                    + hostToolURL.lastPathComponent + " for the build binding")
            return
        }
        let hostToolSHA = SHA256.hash(data: hostToolData)
            .map { String(format: "%02x", $0) }.joined()
        print("SOLVED-POSE-FIXTURE-SIDECAR-TOOL source_sha256=\(hostToolSHA) "
              + "bytes=\(hostToolData.count)")

        // --- NEGATIVE COVERAGE OF THE GATE ITSELF, before any real sidecar is
        // consumed. A run whose comparator fails to reject fabricated bad input
        // is structurally unable to write a fixture.
        guard Self.runSidecarComparatorSelfTest() else { return }
        guard Self.runPersonBoxFlipPin() else { return }

        // Host-side receipts the Simulator cannot compute for itself. The
        // regeneration script exports them; `unset` is recorded honestly.
        // Version/receipt field class: newlines to spaces, `/` PERMITTED, and
        // trimmed. The 2026-08-14 fifteenth-round fixtures were written before
        // the trim was added here and carry a TRAILING SPACE on their
        // `xcode_version` line; that is recorded rather than hand-edited,
        // because a fixture is never hand-edited.
        let macosProduct = Self.receiptSanitized(env["BIOMOTION_FIXTURE_MACOS_PRODUCT"] ?? "unset")
        let macosBuild = Self.receiptSanitized(env["BIOMOTION_FIXTURE_MACOS_BUILD"] ?? "unset")
        let xcodeVersion = Self.receiptSanitized(env["BIOMOTION_FIXTURE_XCODE_VERSION"] ?? "unset")
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

            // --- THE SIDECAR GATE. Fail-closed, tolerance ZERO, BEFORE a single
            // box is consumed and before a fixture byte is written. ------------
            guard let sidecar = sidecarBytes[clipId] else {
                XCTFail("BLOCKED: no sidecar was loaded for \(clipId)")
                return
            }
            let expectation = SidecarPlan.Expectation(
                clip: clipId, videoSHA: source.sha, videoBytes: source.bytes,
                toolSourceSHA: hostToolSHA, timestamps: timestamps,
                duration: duration, fps: fps, step: step,
                start: timestamps.first ?? 0, wanted: wanted, available: available)
            if let refusal = SidecarPlan.admit(sidecar.data, expectation: expectation) {
                print("SOLVED-POSE-FIXTURE-SIDECAR-GATE clip=\(clipId) status=MISMATCH "
                      + refusal.marker)
                XCTFail("""
                    BLOCKED: \(clipId)'s person-box sidecar disagreed with the generator's own \
                    plan or conventions. \(refusal.marker). No box was consumed and no fixture \
                    byte was written. Tolerance is ZERO by registration: a tolerance may only be \
                    registered by a SUCCESSOR round, derived from a measured delta, never here.
                    """)
                return
            }
            let plan = try SidecarPlan.parse(sidecar.data)
            print("SOLVED-POSE-FIXTURE-SIDECAR-GATE clip=\(clipId) status=OK "
                  + "count=\(plan.count) duration_bits=\(plan.durationBits) "
                  + "fps_bits=\(plan.fpsBits)")

            // PATH-LIKE field class: the fixture records the host log's BASENAME
            // only, and a `/` surviving into it is a BLOCK rather than a redaction.
            let sidecarLogBasename = URL(fileURLWithPath: plan.logPath).lastPathComponent
            guard !sidecarLogBasename.contains("/") else {
                XCTFail("BLOCKED: the host-tool log basename still contains a path separator")
                return
            }

            // --- 4-6. Core ML, retarget, and the FOUR (+1) segment breaks. ---
            //
            // THE CONTROL FLOW INVERTS relative to the fourteenth round, and that
            // is the substance of the amendment. E1 used to be read AFTER the
            // model call from `estimate.usedFallbackBBox`; it is now decided
            // BEFORE the call, because calling `estimate()` without an external
            // box falls straight back into the Vision path that THROWS in this
            // host on every frame.
            var slots: [SampledSlot] = []
            var breaks: [(index: Int, kind: VideoBreakKind)] = []
            var samMsTotal = 0.0
            var samCalls = 0
            var census = ["found": 0, "no_observation": 0, "perform_threw": 0, "conversion_nil": 0]
            var contentEqual = 0
            var contentCompared = 0
            var contentDeltas: [String] = []
            var areaFractions: [Double] = []
            var maxCentreDisplacement = 0.0
            var previousCentre: CGPoint?

            for (i, t) in timestamps.enumerated() {
                // E4 — one undecodable timestamp leaves a HOLE in `index`.
                guard let image = try? await decoder.decodeFrame(at: t) else {
                    breaks.append((i, .decodeDrop))
                    continue
                }
                let record = plan.frames[i]

                // Part C, the image-dependent half: the sidecar must describe
                // the frame THIS host decoded, not some other rendering of it.
                guard record.decodedWidth == Int(image.size.width),
                      record.decodedHeight == Int(image.size.height) else {
                    print("SOLVED-POSE-FIXTURE-SIDECAR-GATE clip=\(clipId) status=MISMATCH part=C "
                          + "field=frames[].decoded_size first_bad_index=\(i) "
                          + "generator_value=\(Int(image.size.width))x\(Int(image.size.height)) "
                          + "sidecar_value=\(record.decodedWidth)x\(record.decodedHeight)")
                    XCTFail("BLOCKED: \(clipId) frame \(i) decoded "
                            + "\(Int(image.size.width))x\(Int(image.size.height)) against the "
                            + "sidecar's \(record.decodedWidth)x\(record.decodedHeight)")
                    return
                }
                guard image.imageOrientation == .up else {
                    XCTFail("BLOCKED: \(clipId) frame \(i) decoded with orientation "
                            + "\(image.imageOrientation.rawValue); the host tool passed `.up` to "
                            + "Vision, so any other orientation invalidates the box")
                    return
                }

                // CONTENT BINDING is a RECEIPT, never a gate (registered). A
                // cross-host byte-equality gate would convert a benign
                // macOS-vs-Simulator CoreGraphics colour-pipeline difference
                // into a spurious BLOCK; whether those two produce bit-identical
                // premultipliedLast/deviceRGB bytes from one H.264 frame is
                // UNMEASURED, and this is how it becomes measured at zero
                // blocking risk.
                let localPixels = SAM3DPoseEstimator.sourcePixels(image)
                let localHash = SAM3DPoseEstimator.checksumBytes(localPixels)
                contentCompared += 1
                if localHash == SidecarPlan.fnvOffsetBasis {
                    XCTFail("BLOCKED: \(clipId) frame \(i) hashed the bare FNV offset basis, "
                            + "i.e. no pixels were read at all")
                    return
                }
                if localHash == record.sourceHash {
                    contentEqual += 1
                } else if contentDeltas.count < 8 {
                    let luma = Self.luminanceStats(localPixels)
                    contentDeltas.append(String(format: "%d:dmean=%.4f,dsd=%.4f", i,
                                                luma.mean - record.lumaMean,
                                                luma.sd - record.lumaSD))
                }

                // E1 — decided PRE-HOC from the sidecar. The three reachable
                // causes are `perform_threw`, `no_observation`, and a degenerate
                // rect after the shared flip+clamp; all three map to production's
                // ONE whole-image-fallback exclusion.
                census[record.status, default: 0] += 1
                var usableBox: CGRect?
                if record.status == "found", let raw = record.box {
                    if SAM3DPoseEstimator.personBoxPixels(
                        visionNormalizedBottomLeftOriginYUp: raw,
                        imageSize: image.size) != nil {
                        usableBox = raw
                    } else {
                        census["conversion_nil", default: 0] += 1
                    }
                }
                guard let box = usableBox else {
                    // NOT a hardcoded break: this is the same call production
                    // makes (OfflineSessionRunner:1113) against the same policy,
                    // so the generator follows production instead of drifting.
                    guard OfflineTemporalPolicy.exclusion(source: .video,
                                                          usedFallbackBBox: true) != nil else {
                        XCTFail("""
                            BLOCKED: OfflineTemporalPolicy no longer excludes a video \
                            whole-image fallback. Production would now ANALYSE that frame, and \
                            this generator cannot reproduce that path without a live Vision \
                            backend. Re-register the generator before regenerating a fixture.
                            """)
                        return
                    }
                    breaks.append((i, .fallbackBBox))
                    continue
                }

                areaFractions.append(Double(box.width * box.height))
                let centre = CGPoint(x: box.midX, y: box.midY)
                if let p = previousCentre {
                    maxCentreDisplacement = max(maxCentreDisplacement,
                                                Double(hypot(centre.x - p.x, centre.y - p.y)))
                }
                previousCentre = centre

                let samStart = CFAbsoluteTimeGetCurrent()
                let estimate: SAM3DPoseEstimator.Output
                do {
                    estimate = try await estimator.estimate(uiImage: image,
                                                            personBoxNormalizedBottomLeft: box)
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

                // A REGRESSION TRIPWIRE, and TAUTOLOGICAL inside this generator:
                // no expression on the seam path can yield true, because the
                // nil-box case branched above. Saying so is the point — it is
                // NOT evidence that the box was validated. It is not tautological
                // in the fast-lane seam pin (PersonBoxTests), where the
                // Simulator's throwing Vision makes it a real discriminator.
                XCTAssertFalse(estimate.usedFallbackBBox,
                               "the injected-box path must never report the whole-image fallback")

                // camT OMITTED, matching production's real call site
                // (OfflineSessionRunner:1104-1106); `frame.index` is the DECODER
                // SLOT, never the array position (:1100-1103).
                let bodyFrame = MHRRetarget.makeBodyFrame(jointCoords: estimate.jointCoords,
                                                          timestamp: t,
                                                          frameNumber: i)

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

            let contentBinding = (contentCompared > 0 && contentEqual == contentCompared)
                ? "CONTENT_BINDING_PROVEN" : "CONTENT_BINDING_UNPROVEN"
            let sortedAreas = areaFractions.sorted()
            let areaMin = sortedAreas.first ?? 0
            let areaMax = sortedAreas.last ?? 0
            let areaMedian = sortedAreas.isEmpty ? 0 : sortedAreas[sortedAreas.count / 2]
            print("SOLVED-POSE-FIXTURE-SIDECAR-CONTENT clip=\(clipId) \(contentBinding) "
                  + "frames_equal=\(contentEqual)/\(contentCompared)"
                  + (contentDeltas.isEmpty ? "" : " deltas=[" + contentDeltas.joined(separator: " ") + "]"))
            print("SOLVED-POSE-FIXTURE-SIDECAR-BOX clip=\(clipId) "
                  + String(format: "area_frac_min=%.6f area_frac_median=%.6f area_frac_max=%.6f "
                           + "centre_disp_max=%.6f", areaMin, areaMedian, areaMax,
                           maxCentreDisplacement)
                  + " census=found:\(census["found"] ?? 0),no_observation:\(census["no_observation"] ?? 0)"
                  + ",perform_threw:\(census["perform_threw"] ?? 0)"
                  + ",conversion_nil:\(census["conversion_nil"] ?? 0)")

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
            // POSITIONS captured since 2026-08-21, not just names: the committed
            // fixtures store post-solve DOF angles plus `stature_measured_m` and
            // the nine marker NAMES, and nothing else about the subject's build —
            // so with `scaling_applied false` the subject-vs-model geometry
            // mismatch that next-step 55 is about was UNINSPECTABLE from a
            // fixture. These are the exact positions `scaleModelWithHeight` would
            // have measured had DEVIATION A not (correctly) refused to call it.
            let (scaleMarkerPositions, scaleMarkerNames) =
                MHRRetarget.segmentScaleMarkers(jointCoords: firstUsable.jointCoords)

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

            // --- SEGMENT-SCALE RECEIPT (next-step 55), printed beside the IK
            // residual summaries it has to be read against. -------------------
            //
            // WHY: the fixture records `scaling_applied false`,
            // `stature_measured_m` and the nine marker NAMES, then stores only
            // post-solve DOF angles — so a reader could not tell whether a large
            // `ik_residual_mm_*` is a pose problem or a BUILD problem (this
            // generic `FullBody.osim` being asked to reach a subject it is not
            // shaped like). These are the numerators of exactly the ratios
            // `scaleModelWithHeight:` computes internally: lower = HJC→AJC
            // averaged L/R, upper = SJC→WJC averaged L/R, trunk = MHR_ROOT →
            // shoulder midpoint (NimbleBridge.mm:1017-1059, with the
            // `segLength` lambda at :1000; all re-read 2026-08-21).
            //
            // ⚠️ THE DENOMINATORS ARE NOT REACHABLE FROM SWIFT. The model's own
            // reference lengths are private ivars cached at load
            // (`_loadedLowerReferenceLength`, `_loadedUpperReferenceLength`,
            // `_loadedPelvisTrunkReferenceLength`, `_loadedMHRTrunkReferenceLength`
            // — NimbleBridge.mm:447-450, assigned at :848-851 from body-origin
            // distances femur→talus, humerus→hand and the humerus-midpoint
            // trunk), and `NimbleBridge.h` exposes no accessor for them. The
            // bridge prints all four ITSELF at load time — `NSLog` at
            // NimbleBridge.mm:836-841, "NimbleBridge: Loaded scale references —
            // lower %.4f m, trunk PELVIS %.4f m / MHR_ROOT %.4f m, upper %.4f m"
            // — in this same run, so the ratio is one division away for a reader
            // of this log. It is deliberately NOT computed here against the
            // 0.8061 / 0.5360 / 0.4820 figures quoted in MHRRetarget.swift:403-404:
            // those are PROSE, and a printed ratio derived from a doc comment
            // would be quoted back later as a measurement. `model_total_mass_kg`
            // IS Swift-readable (NimbleBridge.h:328) and is included as the one
            // model-side build scalar this test can derive itself.
            func scaleMarkerXYZ(_ name: String) -> (x: Double, y: Double, z: Double)? {
                guard let i = scaleMarkerNames.firstIndex(of: name),
                      i * 3 + 2 < scaleMarkerPositions.count else { return nil }
                return (Double(scaleMarkerPositions[i * 3]),
                        Double(scaleMarkerPositions[i * 3 + 1]),
                        Double(scaleMarkerPositions[i * 3 + 2]))
            }
            func scaleMarkerDistance(_ a: String, _ b: String) -> Double {
                guard let p = scaleMarkerXYZ(a), let q = scaleMarkerXYZ(b) else { return -1 }
                let dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z
                return (dx * dx + dy * dy + dz * dz).squareRoot()
            }
            let segmentPairs: [(label: String, a: String, b: String, consumer: String)] = [
                ("lower_l", "LHJC", "LAJC", "scaleModelWithHeight_lower_avg_LR"),
                ("lower_r", "RHJC", "RAJC", "scaleModelWithHeight_lower_avg_LR"),
                ("upper_l", "LSJC", "LWJC", "scaleModelWithHeight_upper_avg_LR"),
                ("upper_r", "RSJC", "RWJC", "scaleModelWithHeight_upper_avg_LR"),
                ("hip_width", "LHJC", "RHJC", "layout_only_never_read_as_a_reference"),
                ("shoulder_width", "LSJC", "RSJC", "layout_only_never_read_as_a_reference"),
            ]
            for pair in segmentPairs {
                print("SOLVED-POSE-FIXTURE-VIDEO-SEGSCALE clip=\(clipId) pair=\(pair.label)"
                      + " markers=\(pair.a)-\(pair.b)"
                      + String(format: " target_m=%.6f", scaleMarkerDistance(pair.a, pair.b))
                      + " model_ref_m=UNAVAILABLE_IN_SWIFT ratio=UNAVAILABLE_IN_SWIFT"
                      + " consumer=\(pair.consumer)")
            }
            var trunkTarget = -1.0
            if let root = scaleMarkerXYZ("MHR_ROOT"), let ls = scaleMarkerXYZ("LSJC"),
               let rs = scaleMarkerXYZ("RSJC") {
                let dx = 0.5 * (ls.x + rs.x) - root.x
                let dy = 0.5 * (ls.y + rs.y) - root.y
                let dz = 0.5 * (ls.z + rs.z) - root.z
                trunkTarget = (dx * dx + dy * dy + dz * dz).squareRoot()
            }
            let lowerTarget = 0.5 * (scaleMarkerDistance("LHJC", "LAJC")
                                     + scaleMarkerDistance("RHJC", "RAJC"))
            let upperTarget = 0.5 * (scaleMarkerDistance("LSJC", "LWJC")
                                     + scaleMarkerDistance("RSJC", "RWJC"))
            print("SOLVED-POSE-FIXTURE-VIDEO-SEGSCALE clip=\(clipId) pair=trunk"
                  + " markers=MHR_ROOT-shoulder_mid"
                  + String(format: " target_m=%.6f", trunkTarget)
                  + " model_ref_m=UNAVAILABLE_IN_SWIFT ratio=UNAVAILABLE_IN_SWIFT"
                  + " consumer=scaleModelWithHeight_trunk_MHR_ROOT_variant")
            print("SOLVED-POSE-FIXTURE-VIDEO-SEGSCALE clip=\(clipId) summary"
                  + " markers_n=\(scaleMarkerNames.count)"
                  + String(format: " lower_target_m=%.6f upper_target_m=%.6f trunk_target_m=%.6f"
                           + " stature_m=%.6f model_total_mass_kg=%.6f",
                           lowerTarget, upperTarget, trunkTarget, stature, bridge.totalMass)
                  + " scaling_applied=false"
                  + String(format: " ik_residual_mm_median=%.6f ik_residual_mm_p95=%.6f"
                           + " ik_residual_mm_max=%.6f", median, p95, worst)
                  + " model_refs=see_NimbleBridge_NSLog_Loaded_scale_references_in_this_same_log"
                  + " note=ratios_not_computed_here_because_the_denominators_are_private_ivars")

            var text = ""
            text += "# GENERATED by tools/pose_fixture/regenerate_solved_pose_fixtures.sh --video - do not hand-edit.\n"
            text += "# Raw per-frame IK joint angles for one pinned gait clip, driven through the\n"
            text += "# production offline path with ONE named deviation: video decode ->\n"
            text += "# person box via macOS-host sidecar (macOS Vision, INTERIM provenance) ->\n"
            text += "# SAM3DBodyPose Core ML -> 127 MHR joints -> MHRRetarget -> per-frame IK on\n"
            text += "# one NimbleBridge, sequentially, warm-started frame to frame.\n"
            text += "# The iOS Simulator has NO Vision ML inference backend (measured 2026-08-14:\n"
            text += "# VNDetectHumanRectanglesRequest.perform THREW on 12/12 sampled frames,\n"
            text += "# com.apple.Vision Code=9), so the box is computed on macOS and consumed here.\n"
            text += "# macOS Vision provenance is NOT iOS Vision provenance: these fixtures are an\n"
            text += "# INTERIM substrate and nothing transitioned on them is quotable as\n"
            text += "# device-grade. A future device lane regenerating them triggers a fresh\n"
            text += "# preregistered re-adjudication.\n"
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
            text += "bbox_source macos_vision INTERIM\n"
            text += "person_box_sidecar sha256=\(sidecar.sha),bytes=\(sidecar.data.count),"
            text += "schema=\(plan.schema)\n"
            text += "person_box_host tool_source_sha256=\(hostToolSHA),"
            text += "swift=\(Self.receiptSanitized(plan.swiftVersion)),"
            text += "macos_product=\(Self.receiptSanitized(plan.macosProduct)),"
            text += "macos_build=\(Self.receiptSanitized(plan.macosBuild)),"
            text += "vision_revision=\(plan.revisionUsed)\n"
            text += "person_box_log \(sidecarLogBasename)\n"
            text += "person_box_frames found=\(census["found"] ?? 0),"
            text += "no_observation=\(census["no_observation"] ?? 0),"
            text += "perform_threw=\(census["perform_threw"] ?? 0),"
            text += "conversion_nil=\(census["conversion_nil"] ?? 0)\n"
            text += "person_box_area_frac "
            text += String(format: "min=%.6f,median=%.6f,max=%.6f", areaMin, areaMedian, areaMax)
            text += "\n"
            text += "person_box_centre_disp_max \(String(format: "%.6f", maxCentreDisplacement))\n"
            text += "content_binding \(contentBinding) frames_equal=\(contentEqual)/\(contentCompared)\n"
            text += "sidecar_gate status=OK,count=\(plan.count),"
            text += "duration_bits=\(plan.durationBits),fps_bits=\(plan.fpsBits)\n"
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
                  // BOTH denominators: under the pre-hoc inversion an E1 frame
                  // never calls `estimate()`, so `sam_calls` counts NON-E1 slots
                  // only. The fourteenth round's `sam frame=119/120 mean_ms=4768`
                  // was a mean over all 120 sampled slots; printing one label for
                  // two populations is how a receipt goes quietly stale.
                  + "sam_calls=\(samCalls) sam_slots=\(timestamps.count) "
                  + "ik_ms_per_frame=\(String(format: "%.1f", retained.isEmpty ? 0 : ikMsTotal / Double(retained.count))) "
                  + "ik_rms_mm_median=\(String(format: "%.4f", median)) "
                  + "ik_rms_mm_p95=\(String(format: "%.4f", p95)) "
                  + "ik_rms_mm_max=\(String(format: "%.4f", worst)) "
                  + "stature_m=\(String(format: "%.4f", stature)) scaling_applied=false "
                  + "path=\(url.path)")
        }
    }
}
