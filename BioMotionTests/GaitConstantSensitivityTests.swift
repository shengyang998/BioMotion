import XCTest

@testable import BioMotion

/// What the detector's two constants are worth, measured rather than asserted in
/// prose.
///
/// The criterion this module replaced failed because its constant silently moved
/// the answer — at `frac = 0.08` `video_012` read its left contact 24 % SHORTER
/// than its right and at `frac = 0.25` 7 % LONGER, so the SIGN of the product's
/// headline claim was a function of an arbitrary number. The replacement's
/// constants are not exempt from that scrutiny just because they are physical,
/// so this file sweeps them and pins what moves.
///
/// The result, in one line: **the level constant moves absolute contact time by
/// up to 2 frames across its plausible range, and moves the LEFT/RIGHT RATIO by
/// 0.04-0.25.** That is why the module's contract is a ratio guarded by a
/// resolution gate, and why the resolution is never allowed to be finer than the
/// stride-to-stride scatter that this same sweep produces.
final class GaitConstantSensitivityTests: XCTestCase {

    private func signal(_ clip: String) throws -> GaitSignal {
        let frames = try GaitClipFixture.load(clip, bundle: Bundle(for: type(of: self))).frames
        return try GaitSignal.build(frames: frames)
    }

    private func contactMs(_ s: GaitSignal, band: Double, duty: Double = StanceDetector.assumedDutyFactor)
        -> (left: Double, right: Double) {
        let d = StanceDetector.detect(s, band: band, duty: duty)
        // Off the CLOCK, not by counting surviving samples — see `StanceInterval`.
        func meanMs(_ side: GaitSide) -> Double {
            mean(d.stance[side].map(\.seconds)) * 1000
        }
        return (meanMs(.left), meanMs(.right))
    }

    /// The band sweep, `k = 1.5 … 3.5`, i.e. from a band that keeps 87 % of
    /// genuinely planted frames to one that keeps 99.95 %.
    ///
    /// Tolerance ±0.5 ms on each entry: every value is an integer frame count
    /// times a fixed `dt` averaged over 5-7 contacts, so it is exact up to the
    /// float width of the fixture.
    func testTheBandConstantMovesContactTimeByUpToTwoFrames() throws {
        let table: [String: [(k: Double, left: Double, right: Double)]] = [
            "video_012": [(1.5, 122.222, 133.333), (2.0, 166.667, 152.381),
                          (2.5, 166.667, 161.905), (3.0, 166.667, 171.429),
                          (3.5, 188.889, 190.476)],
            // `video_013` is the clip Vision dropped 3 frames on, so it is the
            // only one whose row moved when contact duration stopped being a
            // count of surviving samples: e.g. k=2.5 left 150.000 → 161.111 ms,
            // because one of its contacts has a hole and really did last 6
            // sampling intervals while only 5 samples came back.
            "video_013": [(1.5, 116.667, 109.524), (2.0, 150.000, 114.286),
                          (2.5, 161.111, 152.381), (3.0, 183.333, 166.667),
                          (3.5, 205.556, 195.238)],
            "video_015": [(1.5, 155.556, 160.000), (2.0, 188.889, 186.667),
                          (2.5, 205.556, 206.667), (3.0, 222.222, 226.667),
                          (3.5, 244.444, 246.667)],
        ]
        for (clip, rows) in table {
            let s = try signal(clip)
            for row in rows {
                let m = contactMs(s, band: row.k)
                XCTAssertEqual(m.left, row.left, accuracy: 0.5, "\(clip) k=\(row.k) left")
                XCTAssertEqual(m.right, row.right, accuracy: 0.5, "\(clip) k=\(row.k) right")
            }
            // The swing across the whole sweep, in frames — the number a reader
            // of a contact time needs in order to know what it is worth.
            let lefts = rows.map(\.left)
            let swing = (lefts.max()! - lefts.min()!) / (s.sampleInterval * 1000)
            XCTAssertGreaterThan(swing, 1.5, "\(clip): the dependence is real and is not hidden")
            XCTAssertLessThan(swing, 3.0, "\(clip): but it is bounded at ~2 frames")
        }
    }

    /// The same sweep judged on the quantity the product actually reports.
    ///
    /// A level error moves BOTH feet the same way, because both feet are
    /// thresholded at one shared level — that is the design reason the level is
    /// pooled rather than per-foot. Measured ratio spreads: 0.177, 0.252, 0.040.
    ///
    /// Two of the three exceed the 0.10 the earlier work pre-registered, and
    /// that is recorded here as a failure of THAT bound rather than argued away:
    /// at 5 frames per contact a single frame is 20 %, so a ratio of two 5-frame
    /// contacts cannot be stable to 0.10 by construction. The bound that
    /// survives is the resolution gate — which for `video_012` is 10.1 %, i.e.
    /// wider than the 2.9 % asymmetry it measures, so the claim is refused.
    func testTheLeftRightRatioIsTheStableQuantityAndStillNotStableToATenth() throws {
        let expected: [String: Double] = ["video_012": 0.177, "video_013": 0.260, "video_015": 0.040]
        for (clip, spread) in expected {
            let s = try signal(clip)
            let ratios = [1.5, 2.0, 2.5, 3.0, 3.5].map { k -> Double in
                let m = contactMs(s, band: k)
                return m.left / m.right
            }
            XCTAssertEqual(ratios.max()! - ratios.min()!, spread, accuracy: 0.02, clip)
        }
        // And the ratio is far more stable than the absolute value it comes
        // from: video_015's contact time swings 2.7 frames over the sweep while
        // its ratio moves 4 %.
        let s = try signal("video_015")
        let m15 = contactMs(s, band: 1.5), m35 = contactMs(s, band: 3.5)
        XCTAssertEqual((m35.left - m15.left) / (s.sampleInterval * 1000), 2.67, accuracy: 0.1)
        XCTAssertEqual(m35.left / m35.right / (m15.left / m15.right), 1.019, accuracy: 0.01)
    }

    /// The duty-factor prior sets how many frames of each cycle are treated as
    /// plateau when the level is estimated. It is the stronger of the two
    /// constants — 0.25 → 0.35 moves `video_012` from 122 to 200 ms — which is
    /// exactly why the module publishes `stanceFrameBudget` beside
    /// `measuredStanceFrames` and refuses the clip when they disagree.
    ///
    /// At 0.30 all three clips are SELF-CONSISTENT: the level was estimated over
    /// 5, 5 and 6 frames per cycle and the contacts then measure 5, 5 and 6.
    ///
    /// **The self-consistency check is WEAK, and this test is the measurement
    /// that says so.** It was written expecting 0.30 to be the fixed point and
    /// its neighbours not to be. It is not: on `video_012`, 0.20, 0.25, 0.30 AND
    /// 0.35 are all fixed points, at 4, 5, 5 and 6 frames of contact — 100, 167,
    /// 167 and 200 ms. Only 0.40 fails, where the level is estimated over 7
    /// frames and the contacts come back 8.
    ///
    /// So `GaitReport.stanceBudgetInconsistent` catches the RUNAWAY direction
    /// and nothing else, and the duty prior is a genuinely load-bearing constant
    /// worth ±1 frame of contact time that this data does not pin. That is
    /// recorded rather than papered over, and it is the second reason (with the
    /// band sweep above) that absolute contact time is not the deliverable.
    func testTheDutyPriorIsAFixedPointButSoAreItsNeighbours() throws {
        for clip in GaitClipFixture.allIds {
            let s = try signal(clip)
            let d = StanceDetector.detect(s, duty: 0.30)
            let measured = median((d.stance.left + d.stance.right).map { Double($0.samples) })
            XCTAssertEqual(Int(measured.rounded()), d.stanceFrameBudget,
                           "\(clip): 0.30 is a fixed point of assumed-vs-measured stance frames")
        }
        let twelve = try signal("video_012")
        func fixedPoint(_ duty: Double) -> (budget: Int, measured: Int, ms: Double) {
            let d = StanceDetector.detect(twelve, duty: duty)
            let all = (d.stance.left + d.stance.right).map { Double($0.samples) }
            return (d.stanceFrameBudget, Int(median(all).rounded()),
                    mean(d.stance.left.map(\.seconds)) * 1000)
        }
        // Four priors, four fixed points, three different answers.
        for (duty, budget, ms) in [(0.20, 4, 100.0), (0.25, 5, 166.7),
                                   (0.30, 5, 166.7), (0.35, 6, 200.0)] {
            let f = fixedPoint(duty)
            XCTAssertEqual(f.budget, budget, "duty \(duty) budget")
            XCTAssertEqual(f.measured, budget, "duty \(duty) is ALSO self-consistent")
            XCTAssertEqual(f.ms, ms, accuracy: 0.5, "duty \(duty) contact")
        }
        // Only the runaway is caught.
        let runaway = fixedPoint(0.40)
        XCTAssertEqual(runaway.budget, 7)
        XCTAssertEqual(runaway.measured, 8)
    }

    /// The stride period is the one estimate a window shift used to break, so
    /// its octave rule gets its own assertion: 18, 18 and 19 frames, and never
    /// the 36 the global-maximum rule returned on three of twelve offsets.
    func testStridePeriodIsTheFundamentalAndNotItsOctave() throws {
        let expected: [String: (frames: Int, correlation: Double)] = [
            "video_012": (18, 0.999), "video_013": (18, 0.743), "video_015": (19, 0.987),
        ]
        for (clip, e) in expected {
            let s = try signal(clip)
            let p = StanceDetector.stridePeriod(s)
            XCTAssertEqual(p.frames, e.frames, clip)
            XCTAssertEqual(p.correlation, e.correlation, accuracy: 0.01, clip)
        }
        // Every 100-frame window of every clip agrees, which is what the
        // per-contact stability in G3 rests on.
        for clip in GaitClipFixture.allIds {
            let frames = try GaitClipFixture.load(clip, bundle: Bundle(for: type(of: self))).frames
            var offset = 0
            while offset + 100 <= frames.count {
                let window = try GaitSignal.build(frames: Array(frames[offset..<(offset + 100)]))
                XCTAssertEqual(StanceDetector.stridePeriod(window).frames,
                               expected[clip]!.frames, "\(clip) offset \(offset)")
                offset += 2
            }
        }
    }
}
