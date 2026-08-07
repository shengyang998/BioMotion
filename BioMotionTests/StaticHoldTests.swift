import Combine
import simd
import XCTest
@testable import BioMotion

/// Covers `StaticHoldDetector` and the gate it drives in `NimbleEngine`.
///
/// The point of the feature: the offline pose source zeroes `global_trans`, so
/// the pelvis is pinned at a model constant in every frame and the body has no
/// global translation. `M·q̈` on that path is computed from motion that did not
/// happen. So we only report muscle magnitudes where the subject was measured
/// to be holding still, and we solve those as statics (q̇ = q̈ = 0).
///
/// Two levels here, deliberately:
///  * `testDetector*` — pure, no model, no solver. These pin the CLASSIFIER
///    semantics, including the exact frames a step of motion contaminates.
///  * `testEngine*` — the real FullBody.osim through `NimbleEngine`, proving a
///    held sequence produces muscle output and a moving one does not.
final class StaticHoldTests: XCTestCase {

    // MARK: - Fixtures

    /// One frame's markers in the flat `[x,y,z,…]` layout the bridge takes.
    private static func flat(_ points: [SIMD3<Double>]) -> [NSNumber] {
        points.flatMap { [NSNumber(value: $0.x), NSNumber(value: $0.y), NSNumber(value: $0.z)] }
    }

    /// `2v/a` — the minimum window span the two physical constants imply.
    /// Derived here for the same reason it is derived in the detector: so an
    /// edit to either constant cannot leave a stale literal behind.
    private static var requiredSpan: Double {
        2 * StaticHoldDetector.holdSpeedThresholdMetersPerSecond
            / StaticHoldDetector.maxDiscardedMeanAccelMetersPerSecondSquared
    }

    private static let threeNames = ["PELVIS", "LKJC", "RKJC"]
    private static let threeAtRest: [SIMD3<Double>] = [
        SIMD3(0, 0.92, 0), SIMD3(0.05, 0.50, -0.09), SIMD3(0.05, 0.50, 0.09),
    ]

    /// The shared 20-marker dancer pose, as `(opensimName, position)`.
    private static var dancerMarkers: [(String, SIMD3<Double>)] {
        OfflineMuscleChainFixture.markers.map { arkitId, opensim, p in
            _ = arkitId
            return (opensim, SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z)))
        }
    }

    /// Feeds `frames` (each a full marker set) at `dt` spacing and returns the
    /// verdict for every push, dated at that push's own timestamp. Classifying
    /// at the newest timestamp rather than at the true Savitzky-Golay centre
    /// only changes the label on the verdict, never the window it examines.
    private func classifyAll(_ frames: [[SIMD3<Double>]],
                             names: [String],
                             dt: TimeInterval) -> [NimbleEngine.MotionClassification] {
        var detector = StaticHoldDetector()
        var out: [NimbleEngine.MotionClassification] = []
        for (i, f) in frames.enumerated() {
            let t = Double(i) * dt
            detector.ingest(flatMarkerPositions: Self.flat(f), markerNames: names, timestamp: t)
            out.append(detector.classify(centeredAt: t))
        }
        return out
    }

    // MARK: - Constants are derived, not tuned

    /// The minimum window span is not an independent knob: it is what the speed
    /// cap and the acceleration budget imply together. If someone edits either
    /// constant, this fails and forces them to redo the arithmetic in
    /// `StaticHoldDetector`'s doc comment.
    ///
    /// ⚠️ The values changed on 2026-08-07 (0.02 m/s / 0.08 m/s² → 0.20 / 0.4905)
    /// and the reason is recorded in that doc comment: 0.08 m/s² is 0.82% of g,
    /// while the muscle QP immediately downstream carries a 0.20-0.35 RELATIVE
    /// torque residual. A term held to 0.8% cannot improve an answer whose
    /// dominant error is 20-35%; it can only refuse frames. The replacement is
    /// stated as a fraction of g so it stays tied to the thing it is being
    /// compared against.
    func testHoldDurationIsImpliedByTheTwoPhysicalConstants() {
        let v = StaticHoldDetector.holdSpeedThresholdMetersPerSecond
        let a = StaticHoldDetector.maxDiscardedMeanAccelMetersPerSecondSquared
        let g = StaticHoldDetector.gravityMetersPerSecondSquared

        XCTAssertEqual(a, StaticHoldDetector.discardedAccelFractionOfG * g, accuracy: 1e-12,
                       "the budget must stay DERIVED from a fraction of g, not typed in")
        XCTAssertEqual(StaticHoldDetector.discardedAccelFractionOfG, 0.05, accuracy: 1e-12,
                       "5% of g — an order of magnitude below the muscle QP's own 20-35% residual")
        XCTAssertEqual(2 * v / a, 0.8154944, accuracy: 1e-6,
                       "2·v/a is the minimum window span implied by the two constants")

        // The speed cap's own derivation: the centrifugal/Coriolis term
        // v²/(g·r) on a 0.4 m segment must stay ~1%, an order of magnitude
        // inside the 5% acceleration budget.
        let velocityTermFraction = v * v / (g * 0.4)
        XCTAssertEqual(velocityTermFraction, 0.0102, accuracy: 5e-4,
                       "the speed cap is set by where v²/(g·r) reaches 1% on a 0.4 m segment")
        XCTAssertLessThan(velocityTermFraction, StaticHoldDetector.discardedAccelFractionOfG,
                          "the velocity term must stay inside the acceleration budget")
    }

    /// The old constants refused frames the pipeline's own error budget had no
    /// reason to refuse. Pin the size of that change so it is a measurement,
    /// not a vibe: at the offline path's 2 fps cadence the admissible peak
    /// marker speed went 2 cm/s → 20 cm/s, because the 4 s window the 9-tap
    /// filter spans there leaves the acceleration bound slack.
    func testBudgetChangeIsTenFoldAtTheOfflineCadence() {
        let dt = 0.5
        let span = Double(SavitzkyGolayFilter.windowSize - 1) * dt   // 4 s
        let a = StaticHoldDetector.maxDiscardedMeanAccelMetersPerSecondSquared
        let allowedBySpan = a * span / 2.0
        let cap = StaticHoldDetector.holdSpeedThresholdMetersPerSecond

        print(String(format: "HOLD-METRIC budget span=%.2fs allowed_by_accel=%.3f m/s cap=%.3f m/s "
                     + "-> binding=%.3f m/s (was 0.020)", span, allowedBySpan, cap, min(allowedBySpan, cap)))

        XCTAssertGreaterThan(allowedBySpan, cap,
                             "at 2 fps the 4 s window makes the speed cap the binding constraint")
        XCTAssertEqual(min(allowedBySpan, cap) / 0.02, 10.0, accuracy: 1e-9,
                       "exactly 10x more permissive than the constant it replaced")
    }

    // MARK: - The measured noise floor, and why the old cap was unreachable

    /// Marker set that contains rigid pairs, so the noise probe has something
    /// to read. `LHJC-RHJC` is the pelvis width; `LHJC-LKJC` the femur.
    private static let rigidNames = ["PELVIS", "LHJC", "RHJC", "LKJC", "RKJC"]
    private static func rigidPose(hipHalf: Double = 0.085, femur: Double = 0.42)
        -> [SIMD3<Double>] {
        [SIMD3(0, 0.924, 0),
         SIMD3(-hipHalf, 0.924, 0), SIMD3(hipHalf, 0.924, 0),
         SIMD3(-hipHalf, 0.924 - femur, 0), SIMD3(hipHalf, 0.924 - femur, 0)]
    }

    /// The floor is MEASURED from distances that physically cannot change, not
    /// assumed. Feed a body whose hip width breathes by a known amount and
    /// assert the reported number is `median|Δd| / (2·dt)`.
    ///
    /// The halving is not cosmetic: a distance change of `d` needs at least
    /// `d/2` of position error on one of the two markers, so this is a rigorous
    /// LOWER bound on the marker speed attributable to noise, which is the only
    /// direction that is safe to be wrong in.
    func testNoiseFloorIsMeasuredFromDistancesThatCannotChange() {
        let dt = 0.1
        let breathe = 0.004     // 4 mm of hip-width wobble per frame
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            let half = 0.085 + (k % 2 == 0 ? 0 : breathe / 2)
            detector.ingest(flatMarkerPositions: Self.flat(Self.rigidPose(hipHalf: half)),
                            markerNames: Self.rigidNames, timestamp: Double(k) * dt)
        }
        let v = detector.classify(centeredAt: 0.8)
        print(String(format: "HOLD-METRIC floor measured=%.5f m/s expected=%.5f m/s peak=%.5f",
                     v.poseNoiseFloorMetersPerSecond, breathe / (2 * dt),
                     v.peakMarkerSpeedMetersPerSecond))
        XCTAssertEqual(v.poseNoiseFloorMetersPerSecond, breathe / (2 * dt), accuracy: 1e-9)
    }

    /// A pose with perfect geometry reports a floor of zero, so real motion is
    /// never excused as noise. Without this the `.indistinguishableFromNoise`
    /// branch could swallow genuine movement.
    func testCleanGeometryReportsZeroNoiseFloor() {
        let dt = 0.1
        let frames = (0..<SavitzkyGolayFilter.windowSize).map { k in
            Self.rigidPose().map { $0 + SIMD3<Double>(0.5 * Double(k) * dt, 0, 0) }
        }
        let v = classifyAll(frames, names: Self.rigidNames, dt: dt).last!
        print("HOLD-METRIC clean-motion floor=\(v.poseNoiseFloorMetersPerSecond) "
            + "peak=\(v.peakMarkerSpeedMetersPerSecond) verdict=\(v.verdict)")
        XCTAssertEqual(v.poseNoiseFloorMetersPerSecond, 0, accuracy: 1e-12,
                       "a rigid translation changes no rigid distance, so it is not noise")
        XCTAssertEqual(v.verdict, .movingBeyondStaticBudget,
                       "50 cm/s with clean geometry is the subject moving, and must say so")
    }

    /// "The subject moved" and "this footage cannot tell us" are different
    /// answers with different remedies, and the old gate collapsed them into
    /// one. When the instrument's own floor is above the stillness bound, the
    /// verdict must name that.
    func testJitterAboveTheBoundReadsAsIndistinguishableNotAsMoving() {
        let dt = 0.1
        let cap = StaticHoldDetector.holdSpeedThresholdMetersPerSecond
        // Hip width alternating by 6 cm per frame: at dt = 0.1 that is a floor
        // of 0.30 m/s, above the 0.20 m/s cap. Deliberately far past anything
        // real footage produces (measured: 3.1 mm median on video_012, i.e. a
        // 4.7 cm/s floor at 30 fps) — the point is to exercise the branch, and
        // the gap between 4.7 and 20 cm/s is the margin the new cap buys.
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            let half = 0.085 + (k % 2 == 0 ? 0 : 0.03)
            detector.ingest(flatMarkerPositions: Self.flat(Self.rigidPose(hipHalf: half)),
                            markerNames: Self.rigidNames, timestamp: Double(k) * dt)
        }
        let v = detector.classify(centeredAt: 0.8)
        print("HOLD-METRIC noisy floor=\(v.poseNoiseFloorMetersPerSecond) "
            + "peak=\(v.peakMarkerSpeedMetersPerSecond) cap=\(cap) verdict=\(v.verdict)")

        XCTAssertGreaterThan(v.poseNoiseFloorMetersPerSecond, cap)
        XCTAssertFalse(v.isHold)
        XCTAssertEqual(v.verdict, .indistinguishableFromNoise)
        XCTAssertTrue(v.verdict.advice.contains("jitters"),
                      "the advice must be about the footage, not about holding still")
    }

    /// A hold stays a hold no matter what the floor says. The floor can only
    /// inflate `peak`, so a peak already inside the budget is inside it a
    /// fortiori — the floor is consulted to EXPLAIN a failure, never to cause
    /// one. Without this ordering, a noisy clip of a genuinely still subject
    /// would lose its muscle output rather than gain an explanation.
    func testNoiseFloorNeverTurnsAHoldIntoAFailure() {
        let dt = 0.1
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            // 2 mm of hip breathing: a real floor, but peak stays under the cap.
            let half = 0.085 + (k % 2 == 0 ? 0 : 0.001)
            detector.ingest(flatMarkerPositions: Self.flat(Self.rigidPose(hipHalf: half)),
                            markerNames: Self.rigidNames, timestamp: Double(k) * dt)
        }
        let v = detector.classify(centeredAt: 0.8)
        print("HOLD-METRIC hold-with-floor floor=\(v.poseNoiseFloorMetersPerSecond) "
            + "peak=\(v.peakMarkerSpeedMetersPerSecond) verdict=\(v.verdict)")
        XCTAssertGreaterThan(v.poseNoiseFloorMetersPerSecond, 0, "precondition: a floor was measured")
        XCTAssertTrue(v.isHold)
        XCTAssertEqual(v.verdict, .hold)
    }

    /// The measured reason the old constant had to go, stated as a comparison
    /// rather than as an opinion.
    ///
    /// Real per-frame rigid-distance drift on `video_012.mov` through the
    /// shipping Core ML model (284 consecutive Vision-detected frames, hip
    /// width, `camt_probe.py` 2026-08-07): median 3.13 mm, p90 6.83 mm, max
    /// 12.07 mm. As a noise floor that is 4.69 cm/s at 30 fps, 4.12 at 10 fps
    /// and 0.59 at 2 fps — so against the OLD 2 cm/s cap the instrument's own
    /// noise exceeded the threshold at every rate above ~3 fps, and a perfectly
    /// still subject could not have been classified as still.
    ///
    /// That is what made the old gate self-defeating: to get under the noise
    /// floor you had to sample slowly, and sampling slowly stretched the 9-tap
    /// window's stillness requirement to four seconds.
    func testMeasuredRealNoiseFloorClearsTheNewCapButNotTheOldOne() {
        let measuredFloorAt30fps = 0.0469     // m/s, from the clip above
        let measuredFloorAt10fps = 0.0412
        let oldCap = 0.02
        let newCap = StaticHoldDetector.holdSpeedThresholdMetersPerSecond

        print(String(format: "HOLD-METRIC real-floor 30fps=%.4f 10fps=%.4f old_cap=%.4f new_cap=%.4f",
                     measuredFloorAt30fps, measuredFloorAt10fps, oldCap, newCap))

        XCTAssertGreaterThan(measuredFloorAt30fps, oldCap,
                             "measured: the old cap sat BELOW the instrument's own noise at 30 fps")
        XCTAssertGreaterThan(measuredFloorAt10fps, oldCap,
                             "and at 10 fps too")
        XCTAssertLessThan(measuredFloorAt30fps, newCap,
                          "the new cap must sit above the measured floor at every usable rate")
        XCTAssertGreaterThan(newCap / measuredFloorAt30fps, 4.0,
                             "with at least 4x of margin, so the verdict is about the subject")
    }

    // MARK: - Detector: the single-photo case

    /// A single photo is one instant. `OfflineSessionRunner` turns it into 9
    /// pushes (4 head pad + 1 + 4 tail pad) of the SAME pose, so every measured
    /// displacement is exactly zero and the frame is a hold. This is the
    /// degenerate case the whole offline photo path depends on.
    func testSinglePhotoIsAHold() {
        // The runner keeps `sampleInterval` at its 1/30 s default for a photo,
        // so the padded window spans 8/30 = 0.267 s — SHORTER than the 2v/a
        // the acceleration budget normally wants. It still passes because the
        // budget is enforced as 2·peak/span and peak is exactly 0.
        let dt = 1.0 / 30.0
        let frames = Array(repeating: Self.threeAtRest, count: SavitzkyGolayFilter.windowSize)
        let verdicts = classifyAll(frames, names: Self.threeNames, dt: dt)
        let last = verdicts[verdicts.count - 1]

        print("HOLD-METRIC photo isHold=\(last.isHold) peak=\(last.peakMarkerSpeedMetersPerSecond) "
            + "window_s=\(last.windowSeconds) samples=\(last.sampleCount) "
            + "implied_accel=\(last.impliedMeanAccelMetersPerSecondSquared)")

        XCTAssertTrue(last.isHold, "a replayed single photo must classify as a hold")
        XCTAssertEqual(last.peakMarkerSpeedMetersPerSecond, 0, accuracy: 1e-12)
        XCTAssertLessThan(last.windowSeconds, Self.requiredSpan,
                          "this case is only interesting because the window is shorter than 2v/a")
        XCTAssertEqual(last.sampleCount, SavitzkyGolayFilter.windowSize)
    }

    /// The very first push has no predecessor, so there is no motion
    /// information at all. That must read as "not a hold", not as "still".
    func testFirstSampleAloneIsNotAHold() {
        var detector = StaticHoldDetector()
        detector.ingest(flatMarkerPositions: Self.flat(Self.threeAtRest),
                        markerNames: Self.threeNames, timestamp: 0)
        let v = detector.classify(centeredAt: 0)
        print("HOLD-METRIC single-sample isHold=\(v.isHold) samples=\(v.sampleCount)")
        XCTAssertFalse(v.isHold, "one sample carries no displacement — absence of measurement is not stillness")
    }

    // MARK: - Detector: the speed boundary

    /// Sweeps across `holdSpeedThresholdMetersPerSecond` with a window long
    /// enough that the acceleration budget is slack, so the speed cap alone
    /// decides, and asserts the verdict is EXACTLY `reportedPeak <= cap` at
    /// every step.
    ///
    /// Asserting against the reported peak rather than against the requested
    /// speed is deliberate. An earlier version of this test asked for a speed
    /// of exactly `cap` by building positions as `speed·k·dt` and letting the
    /// detector divide by `dt` again; the round trip landed on
    /// 0.020000000000000018 m/s, 1.8e-17 ABOVE the cap, and the detector
    /// correctly called it moving. Chasing that would have been testing IEEE
    /// rounding, not the classifier.
    func testSpeedThresholdBoundary() {
        let dt = 0.5   // offline default cadence; 9 samples span 4 s >> 2v/a
        let cap = StaticHoldDetector.holdSpeedThresholdMetersPerSecond

        func drift(speed: Double) -> NimbleEngine.MotionClassification {
            let frames = (0..<SavitzkyGolayFilter.windowSize).map { k in
                Self.threeAtRest.map { $0 + SIMD3<Double>(speed * Double(k) * dt, 0, 0) }
            }
            return classifyAll(frames, names: Self.threeNames, dt: dt).last!
        }

        var flipped = false
        for permille in stride(from: 900, through: 1100, by: 10) {
            let v = drift(speed: cap * Double(permille) / 1000.0)
            let peak = v.peakMarkerSpeedMetersPerSecond
            print(String(format: "HOLD-METRIC boundary requested=%.6f peak=%.17f hold=%@",
                         cap * Double(permille) / 1000.0, peak, v.isHold ? "Y" : "N"))
            XCTAssertEqual(v.isHold, peak <= cap,
                           "verdict must be exactly `peak <= cap` (inclusive) at \(permille)permille")
            // The acceleration budget must not be what is deciding here.
            XCTAssertLessThanOrEqual(v.impliedMeanAccelMetersPerSecondSquared,
                                     StaticHoldDetector.maxDiscardedMeanAccelMetersPerSecondSquared,
                                     "precondition: the 4 s window leaves the accel budget slack")
            if !v.isHold { flipped = true }
        }
        XCTAssertTrue(flipped, "the sweep must actually cross the threshold")
        XCTAssertTrue(drift(speed: cap * 0.9).isHold)
        XCTAssertFalse(drift(speed: cap * 1.1).isHold)
    }

    /// With a window SHORTER than 2v/a the speed cap is no longer sufficient —
    /// the acceleration budget takes over and demands proportionally less
    /// speed. This is what makes a short clip degrade gracefully instead of
    /// either failing outright or silently claiming a bound it cannot support.
    func testShortWindowIsGovernedByTheAccelerationBudget() {
        let dt = 1.0 / 30.0   // 9 samples span 0.267 s, well under 2v/a
        let span = Double(SavitzkyGolayFilter.windowSize - 1) * dt
        let cap = StaticHoldDetector.holdSpeedThresholdMetersPerSecond
        let a = StaticHoldDetector.maxDiscardedMeanAccelMetersPerSecondSquared
        let allowedBySpan = a * span / 2.0   // < cap, because span < 2·cap/a

        func drift(speed: Double) -> NimbleEngine.MotionClassification {
            let frames = (0..<SavitzkyGolayFilter.windowSize).map { k in
                Self.threeAtRest.map { $0 + SIMD3<Double>(speed * Double(k) * dt, 0, 0) }
            }
            return classifyAll(frames, names: Self.threeNames, dt: dt).last!
        }

        XCTAssertLessThan(allowedBySpan, cap,
                          "precondition: at this cadence the span, not the speed cap, is binding")
        let ok = drift(speed: allowedBySpan * 0.98)
        let bad = drift(speed: allowedBySpan * 1.05)
        print("HOLD-METRIC short-window span=\(span) cap=\(cap) allowed_by_span=\(allowedBySpan) "
            + "ok=\(ok.isHold)@accel=\(ok.impliedMeanAccelMetersPerSecondSquared) "
            + "bad=\(bad.isHold)@accel=\(bad.impliedMeanAccelMetersPerSecondSquared)")

        XCTAssertTrue(ok.isHold)
        XCTAssertFalse(bad.isHold,
                       "speed under the speed cap but over the acceleration budget must NOT be a hold")
        XCTAssertLessThan(bad.peakMarkerSpeedMetersPerSecond, cap,
                          "and it must fail on the budget, not on the speed cap")
    }

    /// At a high cadence the 9-sample Savitzky-Golay window spans less than
    /// 2v/a, so the detector reaches further back through its own history. If
    /// it did not, a 60 fps clip would be judged on 0.13 s of evidence.
    func testWindowExtendsBackwardWhenTheFilterWindowIsTooShort() {
        let dt = 1.0 / 60.0
        // Enough history to reach 2v/a = 0.8155 s at 60 fps (49 samples).
        let frames = Array(repeating: Self.threeAtRest, count: 90)
        let v = classifyAll(frames, names: Self.threeNames, dt: dt).last!
        print("HOLD-METRIC extended samples=\(v.sampleCount) window_s=\(v.windowSeconds)")
        XCTAssertGreaterThan(v.sampleCount, SavitzkyGolayFilter.windowSize)
        XCTAssertGreaterThanOrEqual(v.windowSeconds, Self.requiredSpan - 1e-9,
                                    "must reach back to at least 2v/a of evidence")
    }

    // MARK: - Detector: what "peak" means

    /// `peakSpeed` is a MAX over markers on purpose: one limb moving is
    /// motion, and the failure direction we want is "refuse to report muscle",
    /// never "report muscle for a moving subject". The median is exposed
    /// alongside so a one-marker verdict stays diagnosable.
    func testOneMovingMarkerMakesTheWholeFrameMoving() {
        let dt = 0.5
        let frames = (0..<SavitzkyGolayFilter.windowSize).map { k -> [SIMD3<Double>] in
            var f = Self.threeAtRest
            f[1] += SIMD3<Double>(0, 0.25 * Double(k), 0)   // 50 cm/s on one knee only
            return f
        }
        let v = classifyAll(frames, names: Self.threeNames, dt: dt).last!
        print("HOLD-METRIC one-marker isHold=\(v.isHold) peak=\(v.peakMarkerSpeedMetersPerSecond) "
            + "median=\(v.medianMarkerSpeedMetersPerSecond)")
        XCTAssertFalse(v.isHold)
        XCTAssertEqual(v.peakMarkerSpeedMetersPerSecond, 0.50, accuracy: 1e-9)
        XCTAssertEqual(v.medianMarkerSpeedMetersPerSecond, 0.0, accuracy: 1e-9,
                       "median stays at zero — that is the signal that ONE marker moved, not the body")
    }

    /// A marker dropping out of tracking changes the marker SET, not the
    /// subject's position. Differencing only what the two frames have in
    /// common keeps that from reading as a huge displacement.
    func testMarkerSetChangeIsNotMistakenForMotion() {
        var detector = StaticHoldDetector()
        let dt = 0.5
        for k in 0..<SavitzkyGolayFilter.windowSize {
            // Drop the third marker on odd frames.
            let names = k % 2 == 0 ? Self.threeNames : Array(Self.threeNames.prefix(2))
            let pts = k % 2 == 0 ? Self.threeAtRest : Array(Self.threeAtRest.prefix(2))
            detector.ingest(flatMarkerPositions: Self.flat(pts), markerNames: names,
                            timestamp: Double(k) * dt)
        }
        let v = detector.classify(centeredAt: Double(SavitzkyGolayFilter.windowSize - 1) * dt)
        print("HOLD-METRIC dropout isHold=\(v.isHold) peak=\(v.peakMarkerSpeedMetersPerSecond)")
        XCTAssertTrue(v.isHold, "a marker appearing/disappearing is not the subject moving")
    }

    func testResetClearsHistory() {
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            detector.ingest(flatMarkerPositions: Self.flat(Self.threeAtRest),
                            markerNames: Self.threeNames, timestamp: Double(k) * 0.5)
        }
        XCTAssertTrue(detector.classify(centeredAt: 2.0).isHold)
        detector.reset()
        let after = detector.classify(centeredAt: 2.0)
        print("HOLD-METRIC after-reset isHold=\(after.isHold) samples=\(after.sampleCount)")
        XCTAssertFalse(after.isHold)
        XCTAssertEqual(after.sampleCount, 0)
    }

    // MARK: - Detector: a synthetic moving sequence, counted

    /// The headline measurement the brief asks for: build a clip that is still,
    /// then moves, then is still again, and count what gets classified as what.
    ///
    /// The expected split is DERIVED, not observed-then-asserted. At the 2 fps
    /// offline cadence the 9-sample Savitzky-Golay window already spans 4 s, so
    /// no backward extension happens and the examined window for the push at
    /// index `i` is samples `[i-8 … i]`, i.e. transitions `(i-8→i-7) … (i-1→i)`.
    /// A single fast transition `t` therefore contaminates every push in
    /// `[t, t+8]` — which is exactly the point: a frame next to real motion has
    /// no business claiming static equilibrium.
    func testSyntheticMovingSequenceClassification() {
        let dt = 0.5
        let stillA = 12, moving = 8, stillB = 12
        let stepPerFrame = 0.40   // 80 cm/s at this cadence — 4x the 20 cm/s cap

        var frames: [[SIMD3<Double>]] = []
        var pose = Self.threeAtRest
        for _ in 0..<stillA { frames.append(pose) }
        for _ in 0..<moving {
            // Articulated, not rigid: only the knees descend, as they would in
            // a squat. The pelvis marker is stationary — which is exactly the
            // situation on the real path, where the pelvis is pinned by
            // construction and only relative motion is ever observable.
            pose[1] += SIMD3<Double>(0, -stepPerFrame, 0)
            pose[2] += SIMD3<Double>(0, -stepPerFrame, 0)
            frames.append(pose)
        }
        for _ in 0..<stillB { frames.append(pose) }

        let verdicts = classifyAll(frames, names: Self.threeNames, dt: dt)
        let holds = verdicts.filter(\.isHold).count
        let movingCount = verdicts.count - holds

        // Transitions with real motion are at indices stillA … stillA+moving-1.
        // Each contaminates pushes [t, t+windowSize-1]. Pushes before the first
        // full window are also not holds (no measured sample / short history is
        // still fine here since dt is large, but index 0 has no predecessor).
        let firstBad = stillA
        let lastBad = stillA + moving - 1 + (SavitzkyGolayFilter.windowSize - 1)
        var expectedMoving = 0
        for i in 0..<frames.count where i >= firstBad && i <= lastBad { expectedMoving += 1 }
        expectedMoving += 1   // push 0: no predecessor, so no measurement

        print("HOLD-METRIC sequence total=\(verdicts.count) holds=\(holds) moving=\(movingCount) "
            + "expected_moving=\(expectedMoving)")
        for (i, v) in verdicts.enumerated() {
            print(String(format: "HOLD-METRIC frame=%02d t=%.1f hold=%@ peak_cm_s=%.3f window_s=%.1f",
                         i, Double(i) * dt, v.isHold ? "Y" : "N",
                         v.peakMarkerSpeedMetersPerSecond * 100, v.windowSeconds))
        }

        XCTAssertEqual(movingCount, expectedMoving,
                       "the contaminated span is exactly the motion plus one filter window after it")
        XCTAssertGreaterThan(holds, 0, "the still phases must survive as holds")
        // Spot-check the semantics rather than only the totals.
        XCTAssertTrue(verdicts[stillA - 1].isHold, "last frame before any motion is still a hold")
        XCTAssertFalse(verdicts[stillA].isHold, "the first moving frame must not be a hold")
        XCTAssertFalse(verdicts[stillA + moving - 1].isHold, "nor the last moving frame")
        XCTAssertTrue(verdicts[frames.count - 1].isHold, "the clip settles back into a hold")
    }

    // MARK: - Engine: the gate actually changes what is reported

    private func bodyFrame(_ markers: [(String, SIMD3<Double>)],
                           timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        // Back-map OpenSim marker names to the ARKit ids `processFrame` expects.
        let joints: [TrackedJoint] = markers.compactMap { opensim, p in
            guard let m = JointMapping.primary.first(where: { $0.opensimName == opensim }) else { return nil }
            return TrackedJoint(id: m.arkitName, name: m.displayName,
                                worldPosition: SIMD3<Float>(Float(p.x), Float(p.y), Float(p.z)),
                                isTracked: true)
        }
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    @MainActor
    private func submitAndWait(_ engine: NimbleEngine, _ frame: BodyFrame,
                               timeout: TimeInterval = 20) async -> Bool {
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
                .sink { _ in DispatchQueue.main.asyncAfter(deadline: .now() + 0.03) { finish(true) } }
            engine.processFrame(frame)
            DispatchQueue.main.asyncAfter(deadline: .now() + timeout) { finish(false) }
        }
    }

    @MainActor
    private func loadedEngine() async throws -> NimbleEngine {
        let engine = NimbleEngine()
        engine.loadBundledModel()
        let deadline = Date().addingTimeInterval(120)
        while !engine.isModelLoaded && Date() < deadline {
            try? await Task.sleep(nanoseconds: 100_000_000)
        }
        XCTAssertTrue(engine.isModelLoaded, "model never finished loading")
        return engine
    }

    /// A held sequence — the offline runner's own 4 + 1 + 4 cadence on one pose
    /// — must still produce muscle output, and must say it came from a static
    /// solve.
    @MainActor
    func testHeldSequenceProducesStaticHoldMuscleOutput() async throws {
        let engine = try await loadedEngine()
        engine.staticHoldGating = true

        let dt = 0.5
        let markers = Self.dancerMarkers
        for push in 0..<SavitzkyGolayFilter.windowSize {
            let ts = Double(push - SavitzkyGolayFilter.halfWindow) * dt
            _ = await submitAndWait(engine, bodyFrame(markers, timestamp: ts, frameNumber: push))
        }

        let solve = try XCTUnwrap(engine.lastSolve, "no solve published after a full window")
        print("HOLD-METRIC engine-held isHold=\(solve.motion.isHold) "
            + "static=\(solve.isStaticHoldEstimate) muscle=\(solve.muscle != nil) "
            + "peak_cm_s=\(solve.motion.peakMarkerSpeedMetersPerSecond * 100) "
            + "window_s=\(solve.motion.windowSeconds) center=\(solve.centerTimestamp)")

        XCTAssertTrue(solve.motion.isHold, "nine replays of one pose must be a hold")
        XCTAssertTrue(solve.isStaticHoldEstimate, "and must be solved as statics")
        XCTAssertNotNil(solve.muscle, "a hold must still produce muscle output")
        XCTAssertNotNil(solve.id)
        XCTAssertEqual(solve.centerTimestamp, 0.0, accuracy: 1e-6,
                       "the record is dated at the window centre, which the runner routes on")
    }

    /// The load-bearing claim: a MOVING sequence gets pose and no muscle
    /// magnitudes at all.
    ///
    /// The motion here is a whole-body translation, which keeps IK trivially
    /// solvable (it lands on the pelvis free joint) so that what this test
    /// measures is the gate, not the solver's behaviour on an awkward pose.
    /// Marker speed is then exactly `step/dt` for every marker.
    @MainActor
    func testMovingSequenceIsMarkedPoseOnly() async throws {
        let engine = try await loadedEngine()
        engine.staticHoldGating = true

        let dt = 0.5
        let step = 0.50                     // 100 cm/s, 5x the cap
        let base = Self.dancerMarkers
        var sawMuscle = false
        var lastMotion: NimbleEngine.MotionClassification?

        for push in 0..<(SavitzkyGolayFilter.windowSize + 3) {
            let shift = SIMD3<Double>(step * Double(push), 0, 0)
            let markers = base.map { ($0.0, $0.1 + shift) }
            _ = await submitAndWait(engine, bodyFrame(markers, timestamp: Double(push) * dt,
                                                      frameNumber: push))
            if let s = engine.lastSolve {
                lastMotion = s.motion
                if s.muscle != nil { sawMuscle = true }
            }
        }

        let motion = try XCTUnwrap(lastMotion, "no solve published at all")
        print("HOLD-METRIC engine-moving isHold=\(motion.isHold) "
            + "peak_cm_s=\(motion.peakMarkerSpeedMetersPerSecond * 100) "
            + "window_s=\(motion.windowSeconds) sawMuscle=\(sawMuscle) "
            + "lastMuscleResult=\(engine.lastMuscleResult != nil)")

        XCTAssertFalse(motion.isHold, "100 cm/s is not a hold")
        XCTAssertEqual(motion.peakMarkerSpeedMetersPerSecond, step / dt, accuracy: 1e-6,
                       "a rigid translation gives every marker exactly step/dt")
        XCTAssertFalse(sawMuscle,
                       "a moving frame must report pose only — no muscle magnitudes")
        XCTAssertNil(engine.lastSolve?.muscle)
        XCTAssertNil(engine.lastSolve?.id)
    }

    /// Does turning the gate ON change the numbers on the single-photo path
    /// that already worked? This is a CONTROLLED comparison, because a naive
    /// one is confounded.
    ///
    /// ─────────────────────────────────────────────────────────────────────
    /// THE TRIPWIRE BELOW FIRED, AS DESIGNED. Re-derived 2026-08-07.
    /// ─────────────────────────────────────────────────────────────────────
    /// The original reasoning: the offline padding replays one pose, so `ddq`
    /// should be ~1e-16 and zeroing it a no-op — yet peak torque read 75.196 Nm
    /// dynamic vs 75.249 Nm static, a 0.052 Nm gap. The explanation was that
    /// the ENGINE re-solves IK on every push and IK on identical markers did
    /// not return the same answer, so the filter differentiated that drift into
    /// an acceleration the subject never had. Removing that artifact was the
    /// stated point of the feature, and this test asserted the artifact existed
    /// (`maxConsecutive > 0`) so that its disappearance could not go unnoticed.
    ///
    /// It has disappeared. The 2026-08-07 IK work replaced nimble's
    /// error-change termination with a stationarity test, so a repeated solve
    /// on identical markers is now a fixed point. Measured here, same fixture,
    /// same harness:
    ///
    ///   max_from_first_rad        0.0    (was ~0.17)
    ///   max_consecutive_warm_rad  0.0    (was > 0, the artifact)
    ///   dynamicA / dynamicB / static peak torque
    ///                             84.10433817558118 Nm, all three IDENTICAL
    ///   control_delta 0.0 · treatment_delta 0.0 · budget 0.00815
    ///
    /// **Consequence, recorded rather than hidden: static-hold gating is now a
    /// measurable no-op on a hold.** Its remaining value is entirely on the
    /// other branch — refusing to publish muscle magnitudes for a frame where
    /// the subject was MOVING, which the pose source cannot supply the
    /// accelerations for. That is `testMovingSequenceIsMarkedPoseOnly`, and it
    /// is where this feature's justification now lives.
    ///
    /// The assertion is inverted rather than deleted, so a solver regression
    /// that reintroduces the drift fails here and re-opens the question.
    ///
    /// Attribution still needs a control, because `NimbleBridge.mm:296`
    /// documents the skeleton as SHARED across instances and IK warm-starts
    /// from wherever the last run left it. The gate-OFF configuration is run
    /// TWICE and comes back bit-identical.
    @MainActor
    func testStaticSolveEffectOnAStillPoseIsBelowRunToRunVariation() async throws {
        let markers = Self.dancerMarkers
        let dt = 0.5

        // --- Mechanism first: how much does IK move on identical markers? ---
        // During warm-up `lastIKResult` carries RAW (unsmoothed) IK angles, so
        // the first 8 pushes expose the drift directly.
        let probe = try await loadedEngine()
        var raw: [[String: Double]] = []
        for push in 0..<(SavitzkyGolayFilter.windowSize - 1) {
            _ = await submitAndWait(probe, bodyFrame(markers, timestamp: Double(push) * dt,
                                                     frameNumber: push))
            if let ik = probe.lastIKResult { raw.append(ik.jointAngles) }
        }
        // Two numbers, because they mean different things. The first solve of a
        // clip is COLD (no warm start), so measuring from it conflates
        // cold-vs-warm with warm-to-warm drift. The consecutive figure is the
        // one the Savitzky-Golay filter actually differentiates.
        var maxDriftFromFirst = 0.0
        var maxConsecutive = 0.0
        for (k, sample) in raw.enumerated() {
            for (dof, q) in sample {
                if let q0 = raw[0][dof] { maxDriftFromFirst = max(maxDriftFromFirst, abs(q - q0)) }
                if k >= 2, let qPrev = raw[k - 1][dof] {
                    maxConsecutive = max(maxConsecutive, abs(q - qPrev))
                }
            }
        }
        // Finite-difference PROXY for the acceleration the filter manufactures
        // out of that drift — an order of magnitude for attribution, not the
        // filter's own output (which needs 9 raw samples; only 8 are readable
        // before `lastIKResult` switches to smoothed angles).
        print("HOLD-METRIC ik-drift samples=\(raw.count) "
            + "max_from_first_rad=\(maxDriftFromFirst) max_consecutive_warm_rad=\(maxConsecutive) "
            + "implied_ddq_rad_s2=\(maxConsecutive / (dt * dt))")
        // INVERTED 2026-08-07 — see this method's header. IK is now a fixed
        // point on identical markers, so there is no drift for the filter to
        // differentiate and no artifact for this feature to remove on a hold.
        // Asserting EXACT zero keeps the tripwire live in the other direction:
        // any solver change that reintroduces per-solve drift fails here.
        XCTAssertEqual(maxConsecutive, 0, accuracy: 0,
                       "IK started drifting again on identical markers "
                       + "(\(maxConsecutive) rad). The Savitzky-Golay filter differentiates "
                       + "that into an acceleration the subject never had, and the numbers in "
                       + "this method's header were derived assuming it was gone.")
        XCTAssertEqual(maxDriftFromFirst, 0, accuracy: 0,
                       "even the COLD solve used to differ from the warm ones; it no longer "
                       + "does (\(maxDriftFromFirst) rad)")

        func run(gating: Bool) async throws -> (maxTorque: Double, muscles: Int) {
            let engine = try await loadedEngine()
            engine.staticHoldGating = gating
            for push in 0..<SavitzkyGolayFilter.windowSize {
                let ts = Double(push - SavitzkyGolayFilter.halfWindow) * dt
                _ = await submitAndWait(engine, bodyFrame(markers, timestamp: ts, frameNumber: push))
            }
            let solve = try XCTUnwrap(engine.lastSolve)
            XCTAssertEqual(solve.isStaticHoldEstimate, gating,
                           "the fixture is a hold, so gating should decide the solve type")
            let id = try XCTUnwrap(solve.id)
            let maxT = id.jointTorques.values.map(abs).max() ?? 0
            return (maxT, solve.muscle?.activations.count ?? 0)
        }

        let dynamicA = try await run(gating: false)
        let dynamicB = try await run(gating: false)   // control
        let statics  = try await run(gating: true)

        let controlDelta = abs(dynamicB.maxTorque - dynamicA.maxTorque)
        let treatmentDelta = abs(statics.maxTorque - dynamicB.maxTorque)
        print("HOLD-METRIC parity dynamicA=\(dynamicA.maxTorque) dynamicB=\(dynamicB.maxTorque) "
            + "static=\(statics.maxTorque) control_delta=\(controlDelta) "
            + "treatment_delta=\(treatmentDelta) "
            + "muscles A=\(dynamicA.muscles) B=\(dynamicB.muscles) static=\(statics.muscles)")

        XCTAssertEqual(statics.muscles, dynamicB.muscles,
                       "the gate must not change how many muscles are solved for")
        XCTAssertEqual(controlDelta, 0,
                       "two identical gate-OFF runs must agree exactly — without that the "
                       + "treatment delta below cannot be attributed to the gate at all")

        // The tripwire, derived rather than fitted. The detector's whole promise
        // is that on a hold the discarded mean acceleration is under
        // `maxDiscardedMeanAccel` = 5% of g. If zeroing q̇/q̈ moved the peak
        // torque by MORE than that fraction, the removed term was never a small
        // correction and the static reading would be a different answer rather
        // than a cleaner one. Measured on this fixture: 0.052 Nm of 75.2 Nm =
        // 0.07%, an order of magnitude inside the budget.
        let budget = StaticHoldDetector.discardedAccelFractionOfG
        print(String(format: "HOLD-METRIC parity treatment_fraction=%.5f budget_fraction=%.5f",
                     treatmentDelta / dynamicB.maxTorque, budget))
        XCTAssertLessThan(treatmentDelta / dynamicB.maxTorque, budget,
                          "the gate's effect on an already-still pose must stay inside the "
                          + "acceleration budget the hold criterion is built on")
    }
}
