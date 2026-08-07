import XCTest
import simd

@testable import BioMotion

/// Gates G2, G3 and G5 over the three pinned running clips, plus the control
/// that says WHICH of the two criteria a number came from.
///
/// Every assertion here is a measured value with a tolerance argued in the
/// comment above it. Where the module disagrees with the table pinned in
/// `STATUS.md`, the disagreement is asserted as a number rather than smoothed
/// over — a registered gate that fails is a result; a moved goalpost is not.
final class GaitStanceDetectionTests: XCTestCase {

    private func frames(_ clip: String) throws -> [BodyFrame] {
        try GaitClipFixture.load(clip, bundle: Bundle(for: type(of: self))).frames
    }

    private func report(_ clip: String) throws -> GaitReport {
        try GaitAnalysis.analyse(frames: try frames(clip))
    }

    /// `STATUS.md`'s table, milliseconds, left/right.
    private static let pinnedContactMs: [String: (Double, Double)] = [
        "video_012": (167, 167),
        "video_013": (200, 121),
        "video_015": (233, 247),
    ]
    private static let frameMs = 1000.0 / 30.0

    // MARK: - The signal layer

    func testRunningDirectionIsReadFromFootGeometryAndFlipsWithTheClip() throws {
        // The sign is taken from the toe being anterior of the ankle, NOT from
        // the velocity signal it then interprets — so this is an independent
        // input. Two of the owner's clips run one way and one runs the other,
        // which is why a hard-coded sign would have been caught only by luck.
        let signs = try GaitClipFixture.allIds.map { id -> Double in
            try GaitSignal.build(frames: try frames(id)).forwardSign
        }
        XCTAssertEqual(signs[0], 1.0, "video_012 travels along −x, so stance is the +x plateau")
        XCTAssertEqual(signs[1], 1.0, "video_013 travels along −x")
        XCTAssertEqual(signs[2], -1.0, "video_015 travels the other way")

        // And the evidence behind each sign, so a future pose change that
        // shrinks the toe-ankle offset shows up here rather than as a silently
        // inverted analysis. Tolerance 5 % of the value: these are means over
        // ~120 frames of a fixed fixture, so they are exact up to float width.
        let separations = try GaitClipFixture.allIds.map { id -> Double in
            try GaitSignal.build(frames: try frames(id)).forwardSeparationMeters
        }
        XCTAssertEqual(separations[0], 0.0415, accuracy: 0.0021)
        XCTAssertEqual(separations[1], 0.0280, accuracy: 0.0014)
        XCTAssertEqual(separations[2], 0.0276, accuracy: 0.0014)
        for s in separations {
            XCTAssertGreaterThan(s, GaitSignal.minimumForwardSeparationMeters)
        }
    }

    func testPlateauSpeedIsAPlausibleRunningSpeed() throws {
        // The level is the runner's speed, measured from the signal. Nothing in
        // the pipeline told the module how fast the subject was moving, so this
        // is a free check on the whole construction: 15-21 km/h is running.
        // Tolerance ±0.05 m/s — a pinned fixture, so this only moves if the
        // estimator changes.
        let expected = ["video_012": 5.802, "video_013": 5.009, "video_015": 4.108]
        for (clip, v) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.runningSpeedMetersPerSecond, v, accuracy: 0.05, clip)
            XCTAssertGreaterThan(r.runningSpeedMetersPerSecond, 2.5, "\(clip) is not a walk")
            XCTAssertLessThan(r.runningSpeedMetersPerSecond, 12.0, "\(clip) is not faster than Bolt")
        }
    }

    func testTheDetectorNeverReadsFootHeight() throws {
        // The retired criterion was a fraction of the ankle's VERTICAL range,
        // and its constant flipped the sign of the measured asymmetry. This
        // pins that the replacement is immune: raise both feet by an arbitrary
        // per-frame amount and every contact must be unchanged.
        let original = try frames("video_012")
        var lifted: [BodyFrame] = []
        for (i, f) in original.enumerated() {
            let bump = Float(0.13 * sin(Double(i) * 0.7))   // ±13 cm of nonsense
            let joints = f.joints.map { j -> TrackedJoint in
                guard j.id != GaitSignal.JointID.pelvis else { return j }
                var p = j.worldPosition
                p.y += bump
                return TrackedJoint(id: j.id, name: j.name, worldPosition: p, isTracked: j.isTracked)
            }
            lifted.append(BodyFrame(timestamp: f.timestamp, frameNumber: f.frameNumber, joints: joints))
        }
        let before = try GaitAnalysis.analyse(frames: original)
        let after = try GaitAnalysis.analyse(frames: lifted)
        XCTAssertEqual(before.stance.left, after.stance.left)
        XCTAssertEqual(before.stance.right, after.stance.right)
    }

    // MARK: - G2, and the control that attributes it

    /// CONTROL. The retired ankle-height criterion, reimplemented here in the
    /// TEST (it is deliberately not in the shipped module), reproduces
    /// `STATUS.md`'s pinned contact table on this fixture within one frame.
    ///
    /// This is what makes G2 below readable: it proves the fixture, the clock
    /// and the joint columns are the ones the table was measured from, so any
    /// disagreement in G2 is attributable to the CRITERION and to nothing else.
    func testTheRetiredCriterionStillReproducesThePinnedTable() throws {
        for clip in GaitClipFixture.allIds {
            let f = try frames(clip)
            let pinned = Self.pinnedContactMs[clip]!
            let measured = Self.ankleHeightContactMs(f, fraction: 0.16)
            XCTAssertEqual(measured.left, pinned.0, accuracy: Self.frameMs,
                           "\(clip) left: old criterion \(measured.left) vs pinned \(pinned.0)")
            XCTAssertEqual(measured.right, pinned.1, accuracy: Self.frameMs,
                           "\(clip) right: old criterion \(measured.right) vs pinned \(pinned.1)")
        }
    }

    /// G2 — contact time under the shipped (plateau) criterion.
    ///
    /// Asserted at ±0.5 ms, which is 1/67 of a frame: these are integer frame
    /// counts times a fixed `dt`, so anything looser would not notice a whole
    /// contact moving.
    ///
    /// **The gate against the pinned table is 4 of 6 legs.** Deviations, in
    /// frames: 012 L 0.01, R 0.15; 013 L 1.17 ✗, R 0.94; 015 L 0.82, R 1.21 ✗.
    /// Both misses are asserted below at their measured size, because a
    /// disagreement that is pinned cannot drift unnoticed.
    ///
    /// `video_013`'s row moved when contact duration stopped being a count of
    /// surviving samples (150.000 → 161.111 ms left, 147.619 → 152.381 right):
    /// it is the clip Vision dropped 3 frames on, and two of its contacts have a
    /// hole. The other two clips are byte-identical, which is the property that
    /// says the change fixed a defect rather than moved a number.
    func testG2ContactTimeAgainstThePinnedTable() throws {
        let measured: [String: (Double, Double)] = [
            "video_012": (166.667, 161.905),
            "video_013": (161.111, 152.381),
            "video_015": (205.556, 206.667),
        ]
        var within = 0
        for clip in GaitClipFixture.allIds {
            let r = try report(clip)
            let expect = measured[clip]!
            XCTAssertEqual(r.contactSeconds.left * 1000, expect.0, accuracy: 0.5, "\(clip) left")
            XCTAssertEqual(r.contactSeconds.right * 1000, expect.1, accuracy: 0.5, "\(clip) right")

            let pinned = Self.pinnedContactMs[clip]!
            for (m, p) in [(expect.0, pinned.0), (expect.1, pinned.1)]
            where abs(m - p) <= Self.frameMs { within += 1 }
        }
        XCTAssertEqual(within, 4, "4 of 6 legs land within one frame of the pinned table")

        // The two that do not, pinned at their measured size (frames).
        let thirteen = try report("video_013")
        XCTAssertEqual(abs(thirteen.contactSeconds.left * 1000 - 200) / Self.frameMs,
                       1.17, accuracy: 0.05,
                       "video_013 left is 1.17 frames short of the pinned 200 ms")
        let fifteen = try report("video_015")
        XCTAssertEqual(abs(fifteen.contactSeconds.right * 1000 - 247) / Self.frameMs,
                       1.21, accuracy: 0.05,
                       "video_015 right is 1.21 frames short of the pinned 247 ms")
    }

    func testStridePeriodsReproduceThePinnedTable() throws {
        // Stride is measured over ~18 frames rather than ~5, so it is the part
        // of the table both criteria agree on. Pinned: 606/606, 593/415,
        // 647/628 ms. Tolerance one frame, except video_013's right leg, whose
        // pinned 415 ms is not a stride at all (see G7).
        let expected: [String: (Double, Double)] = [
            "video_012": (600.0, 600.0),
            "video_013": (633.3, 566.7),
            "video_015": (640.0, 650.0),
        ]
        for (clip, e) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.strideSeconds.left * 1000, e.0, accuracy: 0.5, "\(clip) left stride")
            XCTAssertEqual(r.strideSeconds.right * 1000, e.1, accuracy: 0.5, "\(clip) right stride")
        }
        for clip in ["video_012", "video_015"] {
            let r = try report(clip)
            let pinned: (Double, Double) = clip == "video_012" ? (606, 606) : (647, 628)
            XCTAssertEqual(r.strideSeconds.left * 1000, pinned.0, accuracy: Self.frameMs)
            XCTAssertEqual(r.strideSeconds.right * 1000, pinned.1, accuracy: Self.frameMs)
        }
    }

    // MARK: - G3

    /// G3 — window-shift invariance, measured PER CONTACT.
    ///
    /// The earlier harness compared the MEAN contact time over each window,
    /// which mixes two different things: a detector that moved, and a window
    /// that simply contains a different set of contacts (with 5- and 6-frame
    /// contacts in the same clip, the mean moves by half a frame for free).
    /// This matches contacts across offsets by their absolute mid-stance frame
    /// and asks the sharper question the task states: does sliding the window
    /// move THE SAME contact by more than one frame?
    ///
    /// Bound: 1 frame, as registered. Measured: `video_012` 0 frames over 11
    /// contacts, `video_015` 1 frame over 10, `video_013` 3 frames over 12 —
    /// and `video_013` is the clip the module refuses (G7).
    func testG3WindowShiftDoesNotMoveAContactByMoreThanOneFrame() throws {
        for clip in ["video_012", "video_015"] {
            let spread = try Self.windowShiftSpread(try frames(clip))
            for (key, range) in spread.sorted(by: { $0.key < $1.key }) {
                XCTAssertLessThanOrEqual(range.max - range.min, 1,
                                         "\(clip) \(key.side.rawValue) contact at ~\(key.mid): "
                                         + "\(range.min)…\(range.max) frames across window offsets")
            }
            XCTAssertGreaterThanOrEqual(spread.count, 9, "\(clip): too few contacts to judge")
        }
    }

    func testG3IsRegisteredAsFailingOnTheClipTheModuleRefuses() throws {
        // Pinned so the refusal and the instability stay tied together: if a
        // later change makes video_013 stable, this test fails and whoever made
        // it stable has to revisit the refusal.
        let spread = try Self.windowShiftSpread(try frames("video_013"))
        let worst = spread.values.map { $0.max - $0.min }.max() ?? 0
        XCTAssertEqual(worst, 3, "video_013's worst contact moves 3 frames with the window")
        let unstable = spread.values.filter { $0.max - $0.min > 1 }.count
        XCTAssertEqual(unstable, 6, "6 of \(spread.count) contacts exceed the 1-frame bound")
    }

    // MARK: - G5

    /// G5 — a stance already in progress when the window opens is NOT a
    /// touchdown.
    ///
    /// `video_012`'s right foot is on the ground at frames 3…6. Opening the
    /// analysis window at frame 5 puts the module in the middle of that
    /// contact. The earlier implementation counted that as a touchdown, and
    /// that single invalid onset is what failed a whole clip.
    func testG5AStanceAlreadyUnderwayIsNotCountedAsATouchdown() throws {
        let all = try frames("video_012")
        let whole = try GaitAnalysis.analyse(frames: all)
        let firstRight = try XCTUnwrap(whole.stance.right.first)
        XCTAssertEqual(firstRight.firstIndex, 3, "fixture drifted; this test picked frame 3 on purpose")

        let midStance = try GaitAnalysis.analyse(frames: Array(all[5...]))

        // 1. Nothing is reported at the window's edge.
        for side in GaitSide.allCases {
            for interval in midStance.stance[side] {
                XCTAssertGreaterThan(interval.firstIndex, 1,
                                     "\(side.rawValue) contact starts at the window edge")
                XCTAssertLessThan(interval.lastIndex, midStance.frameCount - 2)
            }
        }
        // 2. The clipped run was SEEN and DROPPED, not silently missing.
        XCTAssertFalse(midStance.edgeClipped.right.isEmpty,
                       "the run in progress should be recorded as edge-clipped")
        // Index 1, not 0: index 0 carries no velocity at all (a centred
        // difference has no left neighbour there), so the run in progress can
        // only reach as far back as the first sample that has one.
        XCTAssertEqual(midStance.edgeClipped.right.first?.firstIndex, 1)
        XCTAssertTrue(midStance.flags.contains { if case .edgeClippedRunsExcluded = $0 { return true }; return false })

        // 3. Every touchdown the truncated window does report is a touchdown the
        //    whole clip also found — no manufactured onsets.
        let wholeTouchdowns = Set((whole.stance.left + whole.stance.right).map { $0.touchdown.rounded3 })
        for side in GaitSide.allCases {
            for interval in midStance.stance[side] {
                XCTAssertTrue(wholeTouchdowns.contains(interval.touchdown.rounded3),
                              "\(side.rawValue) touchdown at \(interval.touchdown) exists only in the "
                              + "truncated window")
            }
        }
        // 4. And it is not trimmed into the answer: the surviving right-foot
        //    contacts are the interior ones, one fewer than the whole clip has.
        XCTAssertEqual(whole.stance.right.count, 7)
        XCTAssertEqual(midStance.stance.right.count, 6)
    }

    // MARK: - Helpers

    /// The retired criterion, kept only as a control: stance is where the ankle
    /// sits within `fraction` of its own vertical range of the lowest point,
    /// measured relative to the pelvis. Edge-touching runs are dropped, exactly
    /// as `gait_consistency.py` did.
    private static func ankleHeightContactMs(_ frames: [BodyFrame],
                                             fraction: Double) -> (left: Double, right: Double) {
        func series(_ id: String) -> [Double] {
            frames.map { f in
                let pelvis = f.joints.first { $0.id == GaitSignal.JointID.pelvis }!.worldPosition
                let joint = f.joints.first { $0.id == id }!.worldPosition
                return -(Double(joint.y) - Double(pelvis.y))          // drop below the pelvis
            }
        }
        let dt = 1.0 / 30.0
        func contacts(_ drop: [Double]) -> Double {
            let hi = drop.max()!, lo = drop.min()!
            let level = hi - fraction * (hi - lo)
            var runs: [(Int, Int)] = []
            var start: Int? = nil
            for (i, v) in drop.enumerated() {
                if v > level, start == nil { start = i }
                if v <= level, let s = start { runs.append((s, i - 1)); start = nil }
            }
            if let s = start { runs.append((s, drop.count - 1)) }
            let kept = runs.filter { $0.0 > 0 && $0.1 < drop.count - 1 && $0.1 > $0.0 }
            let durations = kept.map { Double($0.1 - $0.0 + 1) * dt * 1000 }
            return mean(durations)
        }
        return (contacts(series(GaitSignal.JointID.ankle(.left))),
                contacts(series(GaitSignal.JointID.ankle(.right))))
    }

    /// A contact's identity across window offsets: which foot, and roughly when.
    struct ContactKey: Hashable, Comparable {
        let side: GaitSide
        let mid: Int
        static func < (a: ContactKey, b: ContactKey) -> Bool { a.mid < b.mid }
    }

    /// Slide a 100-frame window in steps of 2 (9 offsets, matching the earlier
    /// harness's geometry) and collect, per contact, the range of durations it
    /// was measured to have. Contacts are matched across offsets by their
    /// absolute mid-stance frame, allowing ±2 frames of drift in the midpoint.
    static func windowShiftSpread(_ frames: [BodyFrame]) throws
        -> [ContactKey: (min: Int, max: Int)] {
        var seen: [(side: GaitSide, mid: Int, frames: Int)] = []
        let span = min(100, frames.count - 8)
        var offset = 0
        var offsets = 0
        while offset + span <= frames.count && offsets < 9 {
            let window = Array(frames[offset..<(offset + span)])
            if let signal = try? GaitSignal.build(frames: window) {
                let detected = StanceDetector.detect(signal)
                for side in GaitSide.allCases {
                    for interval in detected.stance[side] {
                        seen.append((side, (interval.firstIndex + interval.lastIndex) / 2 + offset,
                                     interval.samples))
                    }
                }
            }
            offset += 2
            offsets += 1
        }
        var out: [ContactKey: (min: Int, max: Int)] = [:]
        var counts: [ContactKey: Int] = [:]
        for entry in seen.sorted(by: { $0.mid < $1.mid }) {
            let key = out.keys.first { $0.side == entry.side && abs($0.mid - entry.mid) <= 2 }
                ?? ContactKey(side: entry.side, mid: entry.mid)
            let existing = out[key] ?? (min: entry.frames, max: entry.frames)
            out[key] = (min: Swift.min(existing.min, entry.frames),
                        max: Swift.max(existing.max, entry.frames))
            counts[key, default: 0] += 1
        }
        // Only contacts the window saw at least three times can say anything
        // about stability.
        return out.filter { counts[$0.key, default: 0] >= 3 }
    }
}

private extension TimeInterval {
    /// Timestamps come from the fixture verbatim, so equality across two
    /// analyses is exact; rounding to the millisecond only guards against a
    /// future change that recomputes them.
    var rounded3: Int { Int((self * 1000).rounded()) }
}
