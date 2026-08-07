import XCTest
@testable import BioMotion

/// The 9-tap Savitzky-Golay window is LONGER than every foot contact this app
/// measures. These tests measure what that costs, prove the replacement is a
/// strict superset of the old behaviour, and pin the numbers in
/// `WindowedDerivativeFilter`'s documentation so they are re-measured on every
/// run instead of being a claim about a script that no longer exists.
final class DerivativeWindowTests: XCTestCase {

    private let dt = 1.0 / 30.0

    // MARK: - The problem, stated in numbers

    /// The whole reason this type exists. Contacts on the owner's three clips
    /// are 167-247 ms; the shipped window spans 267 ms.
    func testTheShippedWindowIsLongerThanEveryContactMeasured() {
        let span = Double(SavitzkyGolayFilter.windowSize - 1) * dt
        XCTAssertEqual(span, 0.2667, accuracy: 1e-4)

        // Measured contact times, ms, from the pinned clips (STATUS.md).
        let contacts: [(String, Double)] = [
            ("video_012 L", 166.7), ("video_012 R", 161.9),
            ("video_013 L", 150.0), ("video_013 R", 147.6),
            ("video_015 L", 205.6), ("video_015 R", 206.7),
        ]
        for (name, ms) in contacts {
            XCTAssertLessThan(ms / 1000, span,
                              "\(name): a \(ms) ms contact is shorter than the 267 ms window, "
                              + "so no stance frame has a neighbourhood free of a touchdown")
        }
        print("FILTER-METRIC nine_tap_span_ms=\(span * 1000) shortest_contact_ms=147.6")
    }

    /// The second-derivative gain table. 3.30 Hz is the step fundamental of a
    /// 606 ms stride — the frequency the whole gait route depends on.
    func testSecondDerivativeGainTable() {
        let probes = [1.0, 2.0, 3.0, 3.30, 5.0, 7.0, 10.0]
        var rows: [(Int, [Double])] = []
        for taps in [9, 7, 5, 3] {
            let f = WindowedDerivativeFilter(taps: taps)
            let gains = probes.map { f.secondDerivativeGain(atHz: $0, sampleInterval: dt) }
            rows.append((taps, gains))
            print("FILTER-METRIC taps=\(taps) order=\(f.order) "
                  + "span_ms=\(f.spanSeconds(sampleInterval: dt) * 1000) "
                  + "gains=\(gains.map { String(format: "%.4f", $0) })")
        }

        // The 9-tap window HALVES the term the route depends on, and inverts
        // its sign above 7 Hz. Both pinned.
        let nine = WindowedDerivativeFilter(taps: 9)
        XCTAssertEqual(nine.secondDerivativeGain(atHz: 3.30, sampleInterval: dt), 0.4894, accuracy: 1e-3)
        XCTAssertLessThan(nine.secondDerivativeGain(atHz: 7.0, sampleInterval: dt), 0,
                          "the shipped window inverts the sign of the second derivative above 7 Hz")

        // The 5-tap replacement passes most of it and never inverts in band.
        let five = WindowedDerivativeFilter(taps: 5)
        XCTAssertEqual(five.secondDerivativeGain(atHz: 3.30, sampleInterval: dt), 0.8348, accuracy: 1e-3)
        for f in stride(from: 1.0, through: 10.0, by: 0.5) {
            XCTAssertGreaterThan(five.secondDerivativeGain(atHz: f, sampleInterval: dt), 0,
                                 "5-tap gain must not invert at \(f) Hz")
        }

        // Shorter is uniformly closer to an ideal differentiator at every probe.
        for (i, _) in probes.enumerated() {
            XCTAssertGreaterThan(rows[2].1[i], rows[0].1[i],
                                 "5 taps must pass more of \(probes[i]) Hz than 9 taps")
        }
    }

    // MARK: - Equivalence with what shipped

    /// At 9 taps / cubic the DERIVED coefficients are the hard-coded ones. This
    /// is what makes the live camera path and the static-hold path provably
    /// unchanged rather than merely untouched.
    func testNineTapCubicReproducesTheShippedCoefficients() {
        let f = WindowedDerivativeFilter(taps: 9)
        XCTAssertEqual(f.order, 3)

        let pos = [-21.0, 14, 39, 54, 59, 54, 39, 14, -21].map { $0 / 231.0 }
        let vel = [86.0, -142, -193, -126, 0, 126, 193, 142, -86].map { $0 / 1188.0 }
        let acc = [28.0, 7, -8, -17, -20, -17, -8, 7, 28].map { $0 / 462.0 }

        for i in 0..<9 {
            XCTAssertEqual(f.posCoefficients[i], pos[i], accuracy: 1e-14, "pos[\(i)]")
            XCTAssertEqual(f.velCoefficients[i], vel[i], accuracy: 1e-14, "vel[\(i)]")
            XCTAssertEqual(f.accCoefficients[i], acc[i], accuracy: 1e-14, "acc[\(i)]")
        }
    }

    /// And the same on filtered OUTPUT, over a signal with curvature, so a sign
    /// or ordering error could not hide inside symmetric coefficients.
    func testNineTapFilterOutputMatchesTheShippedFilterOnRealisticInput() {
        let old = SavitzkyGolayFilter()
        let new = WindowedDerivativeFilter(taps: 9)
        var compared = 0
        var worst = 0.0
        for i in 0..<60 {
            let t = Double(i) * dt
            // Two decades of frequency plus a ramp: nothing symmetric about it.
            let x = 0.3 * sin(2 * .pi * 1.7 * t) + 0.05 * sin(2 * .pi * 9.0 * t) + 0.2 * t
            let a = old.push(x, timestamp: t)
            let b = new.push(x, timestamp: t)
            XCTAssertEqual(a == nil, b == nil, "warm-up must end on the same sample")
            guard let a, let b else { continue }
            compared += 1
            worst = max(worst, abs(a.pos - b.pos))
            worst = max(worst, abs(a.vel - b.vel))
            worst = max(worst, abs(a.acc - b.acc))
            XCTAssertEqual(a.center, b.center, accuracy: 0, "centre timestamp must be identical")
        }
        XCTAssertEqual(compared, 52)
        XCTAssertLessThan(worst, 1e-9, "9-tap replacement diverges from the shipped filter")
        print("FILTER-METRIC nine_tap_equivalence_worst_abs_diff=\(worst) over \(compared) samples")
    }

    // MARK: - Correctness of the shorter windows

    /// A quadratic filter must reproduce a quadratic exactly: `q = a + b·t + c·t²`
    /// has `dq = b + 2c·t` and `ddq = 2c` everywhere.
    func testShortWindowsAreExactOnAQuadratic() {
        for taps in [3, 5, 7] {
            let f = WindowedDerivativeFilter(taps: taps)
            let (a, b, c) = (1.5, -0.7, 2.25)
            var checked = 0
            for i in 0..<20 {
                let t = Double(i) * dt
                guard let out = f.push(a + b * t + c * t * t, timestamp: t) else { continue }
                checked += 1
                XCTAssertEqual(out.acc, 2 * c, accuracy: 1e-8, "taps=\(taps) ddq")
                XCTAssertEqual(out.vel, b + 2 * c * out.center, accuracy: 1e-8, "taps=\(taps) dq")
                XCTAssertEqual(out.pos, a + b * out.center + c * out.center * out.center,
                               accuracy: 1e-8, "taps=\(taps) q")
            }
            XCTAssertEqual(checked, 20 - taps + 1)
        }
    }

    /// A centred window must be odd and in range, whatever it is handed —
    /// including the `1` the gait module reports for `video_013`.
    func testTapCountsAreCoercedToSomethingCentredAndUsable() {
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(1), 3)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(0), 3)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(-4), 3)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(3), 3)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(4), 3)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(5), 5)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(6), 5)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(9), 9)
        XCTAssertEqual(WindowedDerivativeFilter.admissibleTaps(99), 9)
        for requested in -5...99 {
            let t = WindowedDerivativeFilter.admissibleTaps(requested)
            XCTAssertEqual(t % 2, 1, "taps must be odd for a centred window")
            XCTAssertGreaterThanOrEqual(t, 3)
            XCTAssertLessThanOrEqual(t, 9)
        }
    }

    /// A zero-span window cannot produce a derivative, and must return nothing
    /// rather than dividing by it.
    func testDegenerateTimestampsYieldNothing() {
        let f = WindowedDerivativeFilter(taps: 5)
        var last: (pos: Double, vel: Double, acc: Double, center: Double)?
        for _ in 0..<8 { last = f.push(1.0, timestamp: 0) }
        XCTAssertNil(last)

        // Warm-up: nothing at all until the window is full, then every push.
        let g = WindowedDerivativeFilter(taps: 5)
        for i in 0..<8 {
            let out = g.push(Double(i), timestamp: Double(i) * dt)
            XCTAssertEqual(out == nil, i < 4, "sample \(i)")
        }
    }

    // MARK: - The fix, measured on the real clips

    /// The chosen window must FIT inside the shortest contact of each clip —
    /// that is the whole point — and the 9-tap one must not.
    func testTheChosenWindowFitsInsideAContactOnEveryPinnedClip() throws {
        for id in GaitClipFixture.allIds {
            let frames = try GaitClipFixture.load(id, bundle: Bundle(for: type(of: self))).frames
            let report = try GaitAnalysis.analyse(frames: frames)
            let taps = WindowedDerivativeFilter.admissibleTaps(report.filterTapsThatFitOneContact)
            let filter = WindowedDerivativeFilter(taps: taps)
            let span = filter.spanSeconds(sampleInterval: report.sampleInterval)
            let shortestContact = Double(report.filterTapsThatFitOneContact) * report.sampleInterval
            let nineSpan = Double(9 - 1) * report.sampleInterval

            print("FILTER-METRIC clip=\(id) reported_taps=\(report.filterTapsThatFitOneContact) "
                  + "chosen_taps=\(taps) span_ms=\(span * 1000) "
                  + "shortest_contact_ms=\(shortestContact * 1000) nine_span_ms=\(nineSpan * 1000)")

            XCTAssertGreaterThan(nineSpan, shortestContact,
                                 "\(id): the shipped window still overhangs, which is the problem")
            if report.filterTapsThatFitOneContact >= 3 {
                XCTAssertLessThanOrEqual(span, shortestContact + 1e-9,
                                         "\(id): the chosen window must fit inside a contact")
            } else {
                // `video_013` reports 1 tap: even the 3-tap minimum overhangs a
                // 2-frame contact. Recorded as a fact, not smoothed over — and
                // that clip is refused by the gait model anyway.
                XCTAssertGreaterThan(span, shortestContact,
                                     "\(id): recorded as the case the minimum window cannot serve")
            }
        }
    }
}
