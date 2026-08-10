import XCTest

@testable import BioMotion

/// Proves the pinned gait fixtures load, and that the defects they exist to
/// guard against are actually caught.
///
/// Two separate failures made this necessary.
///
/// 1. The extraction defect. `vision_box_probe.swift` wrote PNG names as
///    `t%05.1f.png`, quantising the FILENAME to 0.1 s, so at a 1/30 s step three
///    consecutive records addressed one image: 41 distinct PNGs for 120
///    requested frames. Everything downstream read a 10 Hz staircase wearing a
///    30 fps label, and that staircase produced the "contact is exactly 6
///    frames on 13 of 13 contacts, unchanged across a 2.5x threshold span"
///    result the whole gait route rested on.
///    `testNoTwoAdjacentFramesAreIdentical` is the direct guard.
///
/// 2. The parse defect. The previous fixture's generator wrote numpy reprs and
///    its loader used `Double(f[0])!`, so it trapped on its first data line —
///    which is not a test failure but a dead xctest process, taking all 219
///    other tests with it. The malformed-input tests below assert that every
///    shape of bad input returns a typed error instead.
final class GaitClipFixtureTests: XCTestCase {

    private var bundle: Bundle { Bundle(for: type(of: self)) }

    /// Frame counts as extracted by `labs/sam-3d-body/export/gait_cache.py`.
    /// `video_013` is 119 because `VNDetectHumanRectanglesRequest` returned no
    /// person on 3 frames of the same 4.033 s span.
    private static let expectedFrameCount = ["video_012": 122, "video_013": 119, "video_015": 122]

    private static let nominalDt = 1.0 / 30.0

    // MARK: - The fixture loads

    func testAllThreeClipsLoad() throws {
        let clips = try GaitClipFixture.loadAll(bundle: bundle)
        XCTAssertEqual(clips.map(\.id), GaitClipFixture.allIds)
        for clip in clips {
            XCTAssertFalse(clip.frames.isEmpty, "\(clip.id) loaded with no frames")
        }
    }

    func testFrameCountsAreTheExtractedOnes() throws {
        for id in GaitClipFixture.allIds {
            let clip = try GaitClipFixture.load(id, bundle: bundle)
            XCTAssertEqual(clip.frames.count, Self.expectedFrameCount[id],
                           "\(id) frame count moved; the extraction changed, so every "
                           + "number measured off this clip is stale")
        }
    }

    func testEveryFrameCarriesEveryJoint() throws {
        let expectedSourceMarkers = Dictionary(uniqueKeysWithValues:
            MHRRetarget.table.map { ($0.arkitJointId, $0.opensimMarker) })
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            let firstFrame = try XCTUnwrap(clip.frames.first)
            for joint in firstFrame.joints {
                XCTAssertEqual(joint.opensimMarkerNameOverride,
                               expectedSourceMarkers[joint.id],
                               "\(clip.id) joint \(joint.id) lost the MHR source marker provenance")
            }
            for frame in clip.frames {
                XCTAssertEqual(frame.joints.map(\.id), clip.jointIds,
                               "\(clip.id) frame \(frame.frameNumber) has the wrong joints")
                for joint in frame.joints {
                    XCTAssertTrue(joint.worldPosition.x.isFinite
                                  && joint.worldPosition.y.isFinite
                                  && joint.worldPosition.z.isFinite,
                                  "\(clip.id) frame \(frame.frameNumber) joint \(joint.id) "
                                  + "is not finite")
                }
            }
        }
    }

    // MARK: - The clock

    func testTimestampsAreStrictlyIncreasing() throws {
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            for i in 1..<clip.frames.count {
                let previous = clip.frames[i - 1].timestamp
                let current = clip.frames[i].timestamp
                XCTAssertGreaterThan(current, previous,
                                     "\(clip.id) row \(i): timestamp \(current) does not follow "
                                     + "\(previous); the frames are out of presentation order")
            }
        }
    }

    /// One part in 100 of 1/30 s. Measured margin is thirteen orders of
    /// magnitude better than that (3.5e-15 relative), so this only fires if the
    /// clip was re-extracted at a different rate or the clock was regenerated.
    func testSamplingIntervalIsOneThirtiethOfASecond() throws {
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            let dt = try XCTUnwrap(Self.medianAdjacentInterval(clip),
                                   "\(clip.id) has no adjacent frame pair")
            XCTAssertEqual(dt, Self.nominalDt, accuracy: Self.nominalDt / 100.0,
                           "\(clip.id) base sampling interval is \(dt * 1000) ms")
        }
    }

    /// The median passing does NOT mean the clock is uniform, and one clip's is
    /// not: `video_013` lost 3 frames to a failed person detection, leaving a
    /// 2-frame and a 3-frame hole. Those holes are pinned here so a future
    /// regeneration that quietly regrids onto `i * dt` fails instead of moving
    /// that clip's measured stride from 613 ms to 593 ms.
    func testGapsAreWholeFramesAndTheDroppedFramesAreWhereTheyWere() throws {
        var holesPerClip: [String: [Int]] = [:]
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            let dt = try XCTUnwrap(Self.medianAdjacentInterval(clip))
            var holes: [Int] = []
            for i in 1..<clip.frames.count {
                let gapFrames = (clip.frames[i].timestamp - clip.frames[i - 1].timestamp) / dt
                let whole = gapFrames.rounded()
                XCTAssertEqual(gapFrames, whole, accuracy: 0.2,
                               "\(clip.id) row \(i): a \(gapFrames)-frame gap is not on the "
                               + "sampling grid")
                // The decoder slot the loader read must agree with the clock.
                XCTAssertEqual(Double(clip.frames[i].frameNumber - clip.frames[i - 1].frameNumber),
                               whole, accuracy: 0.0,
                               "\(clip.id) row \(i): frame numbers and timestamps disagree")
                if whole > 1 { holes.append(clip.frames[i - 1].frameNumber) }
            }
            holesPerClip[clip.id] = holes
        }
        XCTAssertEqual(holesPerClip["video_012"], [])
        XCTAssertEqual(holesPerClip["video_015"], [])
        XCTAssertEqual(holesPerClip["video_013"], [24, 29],
                       "video_013's dropped frames moved; its stride numbers are stale")
    }

    // MARK: - The extraction guard

    /// The direct guard against the 0.1 s filename quantisation. Measured
    /// margin: the smallest adjacent per-axis move on any of the three clips is
    /// 0.10 m, so this is nowhere near a borderline assertion — a repeat is a
    /// repeat.
    func testNoTwoAdjacentFramesAreIdentical() throws {
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            var smallestMove = Float.greatestFiniteMagnitude
            for i in 1..<clip.frames.count {
                let previous = clip.frames[i - 1].joints.map(\.worldPosition)
                let current = clip.frames[i].joints.map(\.worldPosition)
                let move = zip(previous, current)
                    .map { max(abs($0.x - $1.x), abs($0.y - $1.y), abs($0.z - $1.z)) }
                    .max() ?? 0
                smallestMove = min(smallestMove, move)
                XCTAssertNotEqual(move, 0,
                                  "\(clip.id) rows \(i - 1) and \(i) are identical. That is the "
                                  + "signature of the extraction defect that produced 41 distinct "
                                  + "images for 120 requested frames.")
            }
            XCTAssertGreaterThan(smallestMove, 0.01,
                                 "\(clip.id) smallest adjacent move is \(smallestMove) m, far "
                                 + "below the 0.10 m measured when this fixture was written")
        }
    }

    // MARK: - The fixture describes the frame it is in

    /// Ties the fixture's columns to the app's own retarget table, so a change
    /// to `MHRRetarget` cannot silently desync from the pinned data.
    func testColumnsAreRetargetJoints() throws {
        let known = Set(MHRRetarget.table.map(\.arkitJointId))
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            XCTAssertEqual(clip.jointIds,
                           ["hips_joint", "left_foot_joint", "right_foot_joint",
                            "left_toes_joint", "right_toes_joint"])
            for id in clip.jointIds {
                XCTAssertTrue(known.contains(id), "\(id) is not in MHRRetarget.table")
            }
            XCTAssertNotNil(clip.frames.first?.rootPosition,
                            "\(clip.id): BodyFrame.rootPosition looks for `hips_joint`")
        }
    }

    /// `MHRRetarget.makeBodyFrame(camT: nil)` pins the raw source root at the model
    /// constant, so these clips carry NO root translation — a consumer that
    /// needs the runner's absolute motion has to get it from `camT`, not from
    /// here. Pinned so that stays visible.
    func testSourceRootIsPinnedAtTheModelConstant() throws {
        for clip in try GaitClipFixture.loadAll(bundle: bundle) {
            for frame in clip.frames {
                let root = try XCTUnwrap(frame.rootPosition)
                XCTAssertEqual(root.x, 0, accuracy: 0)
                XCTAssertEqual(root.y, 0.923987, accuracy: 1e-6)
                XCTAssertEqual(root.z, 0, accuracy: 0)
            }
        }
    }

    // MARK: - Malformed input returns an error instead of trapping

    /// A well-formed two-frame clip, mutated per test below.
    private static let wellFormed = """
        # a comment line
        format biomotion-gait-clip-v1
        clip demo
        frames 2
        joints hips_joint left_foot_joint
        0 0.0 0.0 0.923987 0.0 0.1 0.2 0.3
        1 0.0333333 0.0 0.923987 0.0 0.11 0.21 0.31
        """

    func testTheWellFormedControlParses() throws {
        let clip = try GaitClipFixture.parse(Self.wellFormed)
        XCTAssertEqual(clip.id, "demo")
        XCTAssertEqual(clip.frames.count, 2)
        XCTAssertEqual(clip.frames[1].frameNumber, 1)
        XCTAssertEqual(clip.frames[1].timestamp, 0.0333333, accuracy: 1e-12)
        XCTAssertEqual(clip.frames[0].joints[1].worldPosition, SIMD3<Float>(0.1, 0.2, 0.3))
    }

    /// The exact string that killed the previous fixture. It must come back as
    /// an error; if this test crashes the process instead of failing, the
    /// force-unwrap is back.
    func testNumpyReprIsRejected() {
        let text = Self.wellFormed.replacingOccurrences(of: "\n0 0.0 0.0",
                                                        with: "\n0 np.float64(0.0) 0.0")
        expect(.badDecimal(line: 6, field: 1, text: "np.float64(0.0)"), from: text)
    }

    /// `Double(_: String)` accepts all four of these. None is a metre
    /// coordinate, and a naive `guard let` would pass every one of them into
    /// the gait maths.
    func testStringsSwiftWouldParseButAMetreCoordinateNeverIsAreRejected() {
        for bad in ["nan", "NaN", "inf", "0x1p3", "1.5e-3", "+0.5", ".5", "5."] {
            let text = Self.wellFormed.replacingOccurrences(of: "\n0 0.0 0.0",
                                                            with: "\n0 \(bad) 0.0")
            XCTAssertThrowsError(try GaitClipFixture.parse(text), "`\(bad)` was accepted") { error in
                guard let error = error as? GaitClipFixture.LoadError else {
                    return XCTFail("`\(bad)` gave \(error), not a LoadError")
                }
                XCTAssertEqual(error, .badDecimal(line: 6, field: 1, text: bad))
            }
        }
    }

    func testShortLineIsRejected() {
        let text = Self.wellFormed.replacingOccurrences(of: " 0.1 0.2 0.3", with: " 0.1 0.2")
        expect(.badFieldCount(line: 6, expected: 8, got: 7), from: text)
    }

    func testNonASCIIIsRejected() {
        let text = Self.wellFormed.replacingOccurrences(of: "\n0 0.0", with: "\n0 0\u{00D7}0")
        expect(.notASCII(line: 6, byte: 0xC3), from: text)
    }

    func testDeclaredFrameCountMustMatchTheRows() {
        let text = Self.wellFormed.replacingOccurrences(of: "frames 2", with: "frames 3")
        expect(.frameCountMismatch(declared: 3, found: 2), from: text)
    }

    func testAnotherFormatVersionIsRefused() {
        let text = Self.wellFormed.replacingOccurrences(of: "-clip-v1", with: "-clip-v2")
        expect(.unsupportedFormat(line: 2, got: "biomotion-gait-clip-v2"), from: text)
    }

    func testHeaderOrderIsEnforced() {
        let text = """
            clip demo
            format biomotion-gait-clip-v1
            frames 0
            joints hips_joint
            """
        expect(.badHeaderKey(line: 1, expected: "format", got: "clip"), from: text)
    }

    func testTruncatedHeaderIsRejected() {
        expect(.missingHeaderLine(expectedKey: "joints"),
               from: "format biomotion-gait-clip-v1\nclip demo\nframes 0\n")
    }

    func testEmptyInputIsRejected() {
        expect(.missingHeaderLine(expectedKey: "format"), from: "")
    }

    func testMissingResourceIsAnError() {
        XCTAssertThrowsError(try GaitClipFixture.load("video_999", bundle: bundle)) { error in
            XCTAssertEqual(error as? GaitClipFixture.LoadError,
                           .fileNotFound("Fixtures/gait_video_999.txt"))
        }
    }

    // MARK: - Helpers

    private func expect(_ expected: GaitClipFixture.LoadError,
                        from text: String,
                        file: StaticString = #filePath,
                        line: UInt = #line) {
        XCTAssertThrowsError(try GaitClipFixture.parse(text), file: file, line: line) { error in
            guard let error = error as? GaitClipFixture.LoadError else {
                return XCTFail("got \(error), not a LoadError", file: file, line: line)
            }
            XCTAssertEqual(error, expected, file: file, line: line)
        }
    }

    private static func medianAdjacentInterval(_ clip: GaitClipFixture.Clip) -> Double? {
        guard clip.frames.count >= 2 else { return nil }
        let diffs = (1..<clip.frames.count)
            .map { clip.frames[$0].timestamp - clip.frames[$0 - 1].timestamp }
            .sorted()
        return diffs[diffs.count / 2]
    }
}
