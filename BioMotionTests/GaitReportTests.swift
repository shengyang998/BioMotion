import XCTest
import simd

@testable import BioMotion

/// Gates G4, G6 and G7, the force model's derivation, the two falsifiers, and
/// the one test in this module whose ground truth is known rather than pinned.
final class GaitReportTests: XCTestCase {

    private func frames(_ clip: String) throws -> [BodyFrame] {
        try GaitClipFixture.load(clip, bundle: Bundle(for: type(of: self))).frames
    }

    private func report(_ clip: String) throws -> GaitReport {
        try GaitAnalysis.analyse(frames: try frames(clip))
    }

    private static let frameMs = 1000.0 / 30.0

    // MARK: - G4

    /// CONTROL for G4, the same shape as G2's control: the retired criterion's
    /// stride repeatability reproduces `STATUS.md`'s resolution column
    /// (16 % / 44 % / 6 %) on this fixture, so the difference in the gate below
    /// is the criterion and not the data.
    ///
    /// Tolerance 1 percentage point — the pinned column is quoted to whole
    /// percent.
    func testTheRetiredCriterionStillReproducesThePinnedResolution() throws {
        let expected: [String: Double] = ["video_012": 16, "video_013": 44, "video_015": 6]
        for (clip, pinned) in expected {
            let cv = Self.ankleHeightContactVariationPercent(try frames(clip), fraction: 0.16)
            XCTAssertEqual(max(cv.left, cv.right), pinned, accuracy: 1.0,
                           "\(clip): old criterion \(cv.left)/\(cv.right) vs pinned \(pinned)")
        }
    }

    /// G4 — stride repeatability and the resolution derived from it.
    ///
    /// Asserted at ±0.05 pp (these are exact functions of integer frame counts),
    /// with the pinned column checked separately.
    ///
    /// **The gate against the pinned column is 1 of 3 clips.** `video_013`
    /// reproduces (43.28 % vs 44 %); `video_012` measures 10.14 % against a
    /// pinned 16 % and `video_015` 11.14 % against a pinned 6 %. Both are the
    /// criterion change: the plateau criterion times the FLAT part of stance,
    /// so it counts fewer frames per contact (4.93 and 6.18 rather than ~5 and
    /// ~7.3), which raises the quantisation floor on `video_012` and lets one
    /// 5-frame contact among six 6- and 7-frame ones raise the scatter on
    /// `video_015`. The direction is not uniform, so it is not a bias that can
    /// be divided out — it is reported, not corrected.
    func testG4ResolutionAgainstThePinnedColumn() throws {
        let measured: [String: (framesPerContact: Double, floor: Double,
                                repeatability: Double, resolvable: Double)] = [
            "video_012": (4.9286, 10.145, 7.204, 10.145),
            "video_013": (4.4643, 11.200, 43.279, 43.279),
            "video_015": (6.1833, 8.086, 11.144, 11.144),
        ]
        for (clip, e) in measured {
            let r = try report(clip)
            XCTAssertEqual(r.resolution.framesPerContact, e.framesPerContact, accuracy: 0.01, clip)
            XCTAssertEqual(r.resolution.quantisationFloorPercent, e.floor, accuracy: 0.05, clip)
            XCTAssertEqual(r.resolution.strideRepeatabilityPercent, e.repeatability, accuracy: 0.05, clip)
            XCTAssertEqual(r.resolution.resolvableAsymmetryPercent, e.resolvable, accuracy: 0.05, clip)
            // The published number is never finer than the sampling grid allows.
            XCTAssertGreaterThanOrEqual(r.resolution.resolvableAsymmetryPercent,
                                        r.resolution.quantisationFloorPercent, clip)
        }
        let pinned: [String: Double] = ["video_012": 16, "video_013": 44, "video_015": 6]
        var reproduced = 0
        for (clip, p) in pinned {
            let r = try report(clip)
            if abs(r.resolution.resolvableAsymmetryPercent - p) <= 1.0 { reproduced += 1 }
        }
        XCTAssertEqual(reproduced, 1, "1 of 3 clips reproduces the pinned resolution column")
    }

    func testTheQuantisationFloorIsHalfAFrameOverTheContact() throws {
        // The floor is arithmetic, not a fit: half a frame of edge uncertainty
        // spread over N frames of contact.
        for n in [4.0, 5.0, 6.0, 7.0, 8.3, 12.0] {
            let r = GaitResolution(framesPerContact: n, strideRepeatabilityPercent: 0)
            XCTAssertEqual(r.quantisationFloorPercent, 100 * 0.5 / n, accuracy: 1e-9)
        }
        // The published figures for a 200 ms contact at the four capture rates,
        // which is the sentence the UI will show when it refuses a claim.
        for (fps, expected) in [(30.0, 8.3), (60.0, 4.2), (120.0, 2.1), (240.0, 1.0)] {
            let framesPerContact = 0.200 * fps
            let r = GaitResolution(framesPerContact: framesPerContact, strideRepeatabilityPercent: 0)
            XCTAssertEqual(r.quantisationFloorPercent, expected, accuracy: 0.06, "\(fps) fps")
        }
    }

    func testTheHigherFrameRateAdviceIsArithmeticallyRight() throws {
        let r = try report("video_012")
        let wanted = 5.0                      // a 5 % asymmetry claim
        let needed = r.resolution.framesPerSecondNeeded(for: wanted, currentFPS: r.framesPerSecond)
        XCTAssertEqual(needed, 60.86, accuracy: 0.5,
                       "video_012 would need ~61 fps to resolve 5 %")
        // Self-consistency: at that rate the floor is the claim.
        let scaled = GaitResolution(framesPerContact: r.resolution.framesPerContact
                                        * needed / r.framesPerSecond,
                                    strideRepeatabilityPercent: 0)
        XCTAssertEqual(scaled.quantisationFloorPercent, wanted, accuracy: 1e-6)
    }

    // MARK: - G6

    /// G6 — the resolution number refuses an asymmetry claim finer than itself.
    func testG6AnAsymmetryFinerThanTheResolutionIsRefused() throws {
        let r = try report("video_012")
        // Measured: the two feet differ by 2.90 % of contact time, against a
        // resolution of 10.14 %. So the honest answer is "this clip cannot tell".
        XCTAssertEqual(r.contactAsymmetryPercent, 2.899, accuracy: 0.05)
        XCTAssertFalse(r.resolution.permitsAsymmetryClaim(ofPercent: r.contactAsymmetryPercent))
        XCTAssertNil(r.asymmetryClaim, "a 2.9 % difference must not be published at 10.1 % resolution")
        XCTAssertTrue(r.flags.contains {
            if case .asymmetryBelowResolution = $0 { return true }; return false
        })

        // The gate itself, at and around its own boundary, and symmetric in sign
        // (a left-longer and a right-longer claim of the same size are both
        // either sayable or not).
        let res = r.resolution
        XCTAssertTrue(res.permitsAsymmetryClaim(ofPercent: res.resolvableAsymmetryPercent))
        XCTAssertTrue(res.permitsAsymmetryClaim(ofPercent: -res.resolvableAsymmetryPercent))
        XCTAssertFalse(res.permitsAsymmetryClaim(ofPercent: res.resolvableAsymmetryPercent - 0.001))
        XCTAssertFalse(res.permitsAsymmetryClaim(ofPercent: 0))
        XCTAssertFalse(res.permitsAsymmetryClaim(ofPercent: .nan))
        XCTAssertTrue(res.permitsAsymmetryClaim(ofPercent: 25))

        // And on the other usable clip, where the difference is smaller still.
        let fifteen = try report("video_015")
        XCTAssertEqual(fifteen.contactAsymmetryPercent, -0.539, accuracy: 0.05)
        XCTAssertNil(fifteen.asymmetryClaim)
    }

    func testARefusedClipPublishesNoAsymmetryEvenIfItsNumberIsLarge() throws {
        // video_013 measures a 1.6 % difference, but the point is that a clip
        // whose strides disagree cannot make ANY left/right claim, whatever the
        // number happens to be.
        let r = try report("video_013")
        XCTAssertFalse(r.isUsable)
        XCTAssertNil(r.asymmetryClaim)
    }

    // MARK: - G7

    /// G7 — `video_013` is refused, and each refusal carries its number.
    func testG7TheClipWithDroppedFramesAndA44PercentFloorIsRefused() throws {
        let r = try report("video_013")
        XCTAssertFalse(r.isUsable)

        // 1. Vision lost 3 frames; the gaps are still in the clock.
        XCTAssertTrue(r.flags.contains(.droppedFrames(count: 3, largestGapInFrames: 3)))
        XCTAssertEqual(r.droppedFrameCount, 3)

        // 2. The legs disagree about the stride period by 2.00 frames. In steady
        //    gait the two legs alternate within one cycle, so their stride
        //    periods are the SAME quantity measured twice — this is a genuine
        //    contradiction, not a property of the runner.
        let strideGap = r.refusals.compactMap { refusal -> Double? in
            if case .stridePeriodDisagreesBetweenLegs(let f) = refusal { return f }
            return nil
        }.first
        XCTAssertEqual(try XCTUnwrap(strideGap), 2.0, accuracy: 0.05)

        // 3. Its right leg's stride period varies 18.91 % against a 5.56 % bound.
        let unsteady = r.refusals.compactMap { refusal -> (GaitSide, Double, Double)? in
            if case .strideNotSteady(let s, let p, let b) = refusal { return (s, p, b) }
            return nil
        }
        XCTAssertEqual(unsteady.count, 1)
        XCTAssertEqual(unsteady.first?.0, .right)
        XCTAssertEqual(try XCTUnwrap(unsteady.first?.1), 18.909, accuracy: 0.05)
        XCTAssertEqual(try XCTUnwrap(unsteady.first?.2), 100.0 / 18.0, accuracy: 0.01)

        // 4. Its resolution is the pinned 44 %, i.e. it could not have supported
        //    an asymmetry claim even if nothing else had failed.
        XCTAssertEqual(r.resolution.resolvableAsymmetryPercent, 43.279, accuracy: 0.05)

        // The two clips that pass are not refused, so this is a discriminating
        // test and not a blanket one.
        for clip in ["video_012", "video_015"] {
            XCTAssertTrue(try report(clip).isUsable, "\(clip) should pass every refusal")
        }
    }

    // MARK: - Steadiness

    func testSteadinessVerdictAndItsBound() throws {
        // The bound is one sampling interval of stride period — the coarsest
        // variation this clip could not have distinguished from a steady runner.
        let expected: [String: (left: Double, right: Double, bound: Double, steady: Bool)] = [
            "video_012": (0.000, 0.000, 100.0 / 18.0, true),
            "video_013": (4.707, 18.909, 100.0 / 18.0, false),
            "video_015": (2.083, 2.564, 100.0 / 19.0, true),
        ]
        for (clip, e) in expected {
            let s = try report(clip).steadiness
            XCTAssertEqual(s.strideVariationPercent.left, e.left, accuracy: 0.05, clip)
            XCTAssertEqual(s.strideVariationPercent.right, e.right, accuracy: 0.05, clip)
            XCTAssertEqual(s.boundPercent, e.bound, accuracy: 0.01, clip)
            XCTAssertEqual(s.isSteady, e.steady, clip)
        }
    }

    // MARK: - The force model

    func testForceModelMatchesItsOwnImpulseDerivation() {
        // Independent check of Fmax = (π/2)·m·g·(1 + tf/tc): integrate the
        // half-sine numerically over a stride and require the vertical impulse
        // to equal m·g·T, which is the equation the closed form came from.
        let tc = 0.170, tf = 0.130
        let model = GaitForceModel(contactSeconds: tc, flightSeconds: tf)
        let stride = 2 * (tc + tf)
        let steps = 200_000
        var impulse = 0.0                              // in units of m·g·seconds
        let h = tc / Double(steps)
        for i in 0..<steps {
            let t = (Double(i) + 0.5) * h
            impulse += model.peakVerticalForceInBodyWeights * sin(.pi * t / tc) * h
        }
        impulse *= 2                                   // two contacts per stride
        XCTAssertEqual(impulse, stride, accuracy: 1e-4,
                       "the two contacts must deliver exactly m·g·T of impulse")
    }

    func testForceModelOnTheOwnersClips() throws {
        // Both usable clips land in the physiological range for running
        // (2-3 BW). These are timing-only numbers: no mass, no camera, no depth.
        let expected: [String: (ratio: Double, bw: Double, duty: Double)] = [
            "video_012": (0.8261, 2.8684, 0.2738),
            "video_015": (0.5647, 2.4578, 0.3196),
        ]
        for (clip, e) in expected {
            let f = try report(clip).force
            XCTAssertEqual(f.flightToContactRatio, e.ratio, accuracy: 0.005, clip)
            XCTAssertEqual(f.peakVerticalForceInBodyWeights, e.bw, accuracy: 0.01, clip)
            XCTAssertEqual(f.dutyFactor, e.duty, accuracy: 0.005, clip)
            XCTAssertTrue(f.describesRunning, clip)
            XCTAssertGreaterThan(f.peakVerticalForceInBodyWeights, 2.0, clip)
            XCTAssertLessThan(f.peakVerticalForceInBodyWeights, 3.5, clip)
        }
    }

    func testWalkingIsRefusedBecauseTheImpulseModelDoesNotApply() {
        // No flight phase: the derivation's "F = 0 during flight" step is false,
        // and with two feet down the impulse is not one foot's to carry.
        let walking = GaitForceModel(contactSeconds: 0.62, flightSeconds: -0.02)
        XCTAssertFalse(walking.describesRunning)
        let doubleSupport = GaitForceModel(contactSeconds: 0.60, flightSeconds: 0.0)
        XCTAssertFalse(doubleSupport.describesRunning)
    }

    // MARK: - The falsifier

    /// The requirement that something computed can DISAGREE with the model.
    ///
    /// Flight time is estimated twice from different things: once by OBSERVING
    /// the gap between one foot's last stance sample and the other's touchdown,
    /// and once by CLOSING the stride, `(T − tcL − tcR)/2`. Nothing forces them
    /// to agree — the first uses the ordering of events, the second uses the
    /// stride period and both contact durations — so their difference is a real
    /// residual and it gates the report.
    func testFlightTimeIsCrossCheckedAgainstStrideClosure() throws {
        let expected: [String: (measuredMs: Double, modelledMs: Double, disagreementFrames: Double)] = [
            "video_012": (136.111, 135.714, 0.0119),
            "video_013": (121.212, 151.190, 0.8994),
            "video_015": (116.667, 116.389, 0.0083),
        ]
        for (clip, e) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.measuredFlightSeconds * 1000, e.measuredMs, accuracy: 0.5, clip)
            XCTAssertEqual(r.modelledFlightSeconds * 1000, e.modelledMs, accuracy: 0.5, clip)
            XCTAssertEqual(r.flightDisagreementFrames, e.disagreementFrames, accuracy: 0.02, clip)
        }
        // The two clips that agree do so to a hundredth of a frame — which is
        // the evidence that the periodic model actually describes them, and it
        // is evidence that could have come out otherwise.
        XCTAssertLessThan(try report("video_012").flightDisagreementFrames, 0.05)
        XCTAssertLessThan(try report("video_015").flightDisagreementFrames, 0.05)
    }

    func testTheFalsifierCanActuallyFire() throws {
        // Delete one foot's contact from the middle of a clip by holding that
        // foot still for one cycle: the stride period doubles for that leg while
        // the other is unchanged, so the legs must contradict each other. If
        // this passes silently, the refusal path is decorative.
        let original = try frames("video_012")
        let hold = 48...58                       // one left contact and its edges
        var edited: [BodyFrame] = []
        for (i, f) in original.enumerated() {
            guard hold.contains(i) else { edited.append(f); continue }
            let donor = original[hold.lowerBound - 1]
            let joints = f.joints.map { j -> TrackedJoint in
                guard j.id == GaitSignal.JointID.ankle(.left)
                        || j.id == GaitSignal.JointID.toe(.left) else { return j }
                let frozen = donor.joints.first { $0.id == j.id }!.worldPosition
                return TrackedJoint(id: j.id, name: j.name, worldPosition: frozen, isTracked: true)
            }
            edited.append(BodyFrame(timestamp: f.timestamp, frameNumber: f.frameNumber, joints: joints))
        }
        let r = try GaitAnalysis.analyse(frames: edited)
        XCTAssertFalse(r.isUsable, "a leg that misses a whole contact must not pass")
        XCTAssertFalse(r.refusals.isEmpty)
    }

    // MARK: - Ground truth that is known, not pinned

    /// Every other number in this file is pinned against the fixture, which
    /// proves reproducibility and nothing about accuracy. This one builds a
    /// runner whose contact time is KNOWN by construction and asks what the
    /// module reports.
    ///
    /// **Measured: contact time carries about one frame of bias, and the bias
    /// CHANGES SIGN with the plateau scatter.** At a scatter of 0.15 m/s (a
    /// well-tracked joint) the 2.5σ band is narrow, the two boundary frames —
    /// whose centred differences straddle touchdown and toe-off — fall outside
    /// it, and a true 200 ms contact reads 172 ms at 30 fps. At 0.80 m/s (what
    /// the owner's own clips measure: 0.77-1.05) the band is wide enough to
    /// admit ramp frames and the same contact reads 228 ms.
    ///
    /// So absolute contact time is good to roughly ±28 ms and no better, and
    /// the sign of the error depends on how noisy the reconstruction is. That
    /// is precisely why this module's deliverable is ratios and a resolution
    /// gate rather than a calibrated number: the bias is a common scale on both
    /// feet (172.2 vs 176.2 ms, 227.8 vs 233.3 ms — the two feet move together),
    /// so it largely cancels in left/right and stride-to-stride comparisons.
    ///
    /// **A finding for whoever raises the capture rate:** that ±28 ms is a bound
    /// in TIME and it does not shrink with the frame rate. At 120 fps and a
    /// 0.80 m/s scatter the same 26 ms offset is 3.2 frames. Filming faster buys
    /// PRECISION — the quantisation floor really does fall 9.6 % → 2.1 % — and
    /// it does not buy accuracy of the absolute contact time.
    ///
    /// Tolerance ±0.5 frame throughout: the reported value is a mean over 6-7
    /// integer frame counts, so a single contact flipping by one frame moves it
    /// by 1/6 of a frame, and that is what a cross-platform difference in the
    /// last bit of a `sin` could do.
    func testSyntheticRunnerWithAKnownContactTime() throws {
        let contact = 0.200, flight = 0.120, speed = 4.0
        let cases: [(fps: Double, scatter: Double, left: Double, right: Double)] = [
            (30, 0.15, 172.2, 176.2),
            (120, 0.15, 201.4, 201.2),
            (30, 0.80, 227.8, 233.3),
            (120, 0.80, 226.4, 225.0),
        ]
        for c in cases {
            let f = Self.syntheticRunner(fps: c.fps, contact: contact, flight: flight,
                                         speed: speed, strides: 7, velocityNoise: c.scatter)
            let r = try GaitAnalysis.analyse(frames: f)
            let frameMs = 1000 / c.fps
            let label = "\(Int(c.fps)) fps, scatter \(c.scatter)"
            XCTAssertEqual(r.contactSeconds.left * 1000, c.left, accuracy: 0.5 * frameMs,
                           "\(label) left")
            XCTAssertEqual(r.contactSeconds.right * 1000, c.right, accuracy: 0.5 * frameMs,
                           "\(label) right")
            // Whatever the sign, the error stays inside ~28 ms — and note that
            // is a bound in TIME, not in frames: at 120 fps the same 26 ms is
            // 3.2 frames. Filming faster buys PRECISION (the resolution below),
            // not accuracy of the absolute contact time, whose offset is set by
            // the width of the velocity band against the ramp's duration.
            let errorMs = abs(200 - r.contactSeconds.left * 1000)
            XCTAssertLessThanOrEqual(errorMs, 30, "\(label): |bias| under 30 ms")
            // And the runner's own speed comes back, which nothing told it.
            XCTAssertEqual(r.runningSpeedMetersPerSecond, speed, accuracy: 0.2, label)
        }
        // The sign flip itself, asserted rather than described.
        let quiet = try GaitAnalysis.analyse(frames: Self.syntheticRunner(
            fps: 30, contact: contact, flight: flight, speed: speed, strides: 7,
            velocityNoise: 0.15))
        let noisy = try GaitAnalysis.analyse(frames: Self.syntheticRunner(
            fps: 30, contact: contact, flight: flight, speed: speed, strides: 7,
            velocityNoise: 0.80))
        XCTAssertLessThan(quiet.contactSeconds.left, 0.200, "narrow band reads short")
        XCTAssertGreaterThan(noisy.contactSeconds.left, 0.200, "wide band reads long")

        // The published resolution tracks the frame rate, which is the whole
        // basis of telling the user to film faster.
        let fast = try GaitAnalysis.analyse(frames: Self.syntheticRunner(
            fps: 120, contact: contact, flight: flight, speed: speed, strides: 7,
            velocityNoise: 0.15))
        XCTAssertEqual(quiet.resolution.quantisationFloorPercent, 9.57, accuracy: 0.5)
        XCTAssertEqual(fast.resolution.quantisationFloorPercent, 2.07, accuracy: 0.2)
        XCTAssertLessThan(fast.resolution.resolvableAsymmetryPercent,
                          quiet.resolution.resolvableAsymmetryPercent)
    }

    // MARK: - The filter window this module has to hand the engine

    func testTheEngineNineTapWindowDoesNotFitInsideAContact() throws {
        // The 9-tap centred Savitzky-Golay window spans 8·dt = 267 ms at 30 fps.
        // Every contact in the owner's footage is shorter than that, so no
        // stance frame has a window free of a touchdown or a toe-off. This
        // publishes the largest window that WOULD fit, per clip.
        let expected: [String: Int] = ["video_012": 3, "video_013": 1, "video_015": 5]
        for (clip, taps) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.filterTapsThatFitOneContact, taps, clip)
            XCTAssertFalse(r.nineTapFilterFitsOneContact, clip)
            XCTAssertEqual(r.nineTapFilterSpanSeconds, 8.0 / 30.0, accuracy: 1e-6, clip)
        }
        // At 120 fps a 200 ms contact spans 24 frames, and the 9-tap window
        // fits with room to spare — the same lever as the resolution.
        let fast = try GaitAnalysis.analyse(frames: Self.syntheticRunner(
            fps: 120, contact: 0.200, flight: 0.120, speed: 4.0, strides: 7,
            velocityNoise: 0.15))
        XCTAssertGreaterThanOrEqual(fast.filterTapsThatFitOneContact, 9)
        XCTAssertTrue(fast.nineTapFilterFitsOneContact)
    }

    // MARK: - Input validation

    func testShortOrMalformedInputIsAnErrorRatherThanAGuess() throws {
        let all = try frames("video_012")
        XCTAssertThrowsError(try GaitAnalysis.analyse(frames: Array(all.prefix(10)))) { error in
            XCTAssertEqual(error as? GaitSignal.Failure,
                           .tooFewFrames(count: 10, needed: GaitSignal.minimumFrames))
        }
        let stripped = all.map { f in
            BodyFrame(timestamp: f.timestamp, frameNumber: f.frameNumber,
                      joints: f.joints.filter { $0.id != GaitSignal.JointID.toe(.right) })
        }
        XCTAssertThrowsError(try GaitAnalysis.analyse(frames: stripped)) { error in
            XCTAssertEqual(error as? GaitSignal.Failure,
                           .missingJoint("right_toes_joint", frameIndex: 0))
        }
        let frozenClock = all.map {
            BodyFrame(timestamp: 1.0, frameNumber: $0.frameNumber, joints: $0.joints)
        }
        XCTAssertThrowsError(try GaitAnalysis.analyse(frames: frozenClock)) { error in
            XCTAssertEqual(error as? GaitSignal.Failure, .timestampsNotIncreasing(frameIndex: 1))
        }
    }

    func testTheStanceFrameBudgetAgreesWithWhatTheContactsMeasure() throws {
        // The level is estimated over an assumed number of plateau frames per
        // cycle. That assumption is published and checked: all three clips are
        // self-consistent, so the 0.30 duty prior is confirmed by the data
        // rather than merely believed.
        let expected: [String: Int] = ["video_012": 5, "video_013": 5, "video_015": 6]
        for (clip, n) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.stanceFrameBudget, n, clip)
            XCTAssertEqual(r.measuredStanceFrames, n, clip)
            XCTAssertFalse(r.refusals.contains {
                if case .stanceBudgetInconsistent = $0 { return true }; return false
            }, clip)
        }
    }

    // MARK: - Fixtures

    /// A runner with a known contact time. The foot is EXACTLY stationary in the
    /// ground frame during stance and swings forward on a raised cosine (zero
    /// velocity at both ends, so the plateau's edges are gradual as they are in
    /// real footage rather than a step the detector could not miss). The pelvis
    /// is pinned at the model constant, as the real reconstruction is, so the
    /// module sees the same kind of input it sees in production.
    ///
    /// - Parameter velocityNoise: the plateau scatter to synthesise, m/s. It is
    ///   injected as a deterministic sum of two incommensurate sinusoids whose
    ///   AMPLITUDE scales with `dt`, so the velocity noise is the same at every
    ///   sampling rate and the only thing the frame-rate sweep varies is the
    ///   grid. Without any noise the band `V − 2.5σ` collapses to `V` exactly
    ///   and membership turns on the last bit of a floating-point division —
    ///   which is not a property of any real clip.
    static func syntheticRunner(fps: Double, contact: Double, flight: Double,
                                speed: Double, strides: Int,
                                velocityNoise: Double) -> [BodyFrame] {
        let stride = 2 * (contact + flight)
        let dt = 1 / fps
        let count = Int((Double(strides) * stride / dt).rounded())
        let noiseAmplitude = velocityNoise * dt
        var out: [BodyFrame] = []
        out.reserveCapacity(count)
        for i in 0..<count {
            let t = Double(i) * dt
            var joints = [TrackedJoint(id: GaitSignal.JointID.pelvis, name: "Pelvis",
                                       worldPosition: SIMD3<Float>(0, 0.923_987, 0), isTracked: true)]
            for (index, (side, phase)) in [(GaitSide.left, 0.0),
                                           (GaitSide.right, 0.5 * stride)].enumerated() {
                // Cycle index and phase. The world position ACCUMULATES one
                // stride length per cycle — resetting it each cycle would put a
                // stride-length step into the trajectory every 640 ms.
                let cycles = ((t - phase) / stride).rounded(.down)
                let u = (t - phase) - cycles * stride
                let strideLength = speed * stride
                let worldX: Double
                if u <= contact {
                    worldX = cycles * strideLength                // planted
                } else {
                    let s = (u - contact) / (stride - contact)
                    worldX = cycles * strideLength + strideLength * 0.5 * (1 - cos(.pi * s))
                }
                // Pelvis-relative: the pelvis advances at `speed`, and the
                // reconstruction pins it, so the foot slides backwards.
                let jitter = noiseAmplitude
                    * (sin(1.7 * Double(i) + 0.3 * Double(index))
                       + sin(2.9 * Double(i) + 1.1 + 0.7 * Double(index))) / 2
                let relX = Float(worldX - speed * t + jitter)
                let height = Float(0.08 + 0.30 * max(0, sin(.pi * min(1, max(0, (u - contact)
                                                                            / (stride - contact))))))
                joints.append(TrackedJoint(id: JointIDAnkle(side), name: "ankle",
                                           worldPosition: SIMD3<Float>(relX, height - 0.92, 0),
                                           isTracked: true))
                joints.append(TrackedJoint(id: JointIDToe(side), name: "toe",
                                           worldPosition: SIMD3<Float>(relX + 0.15, height - 0.95, 0),
                                           isTracked: true))
            }
            out.append(BodyFrame(timestamp: 3.0 + t, frameNumber: i, joints: joints))
        }
        return out
    }

    private static func JointIDAnkle(_ s: GaitSide) -> String { GaitSignal.JointID.ankle(s) }
    private static func JointIDToe(_ s: GaitSide) -> String { GaitSignal.JointID.toe(s) }

    /// The retired criterion's per-side stride repeatability, for the control.
    static func ankleHeightContactVariationPercent(_ frames: [BodyFrame], fraction: Double)
        -> (left: Double, right: Double) {
        func cv(_ id: String) -> Double {
            let drop: [Double] = frames.map { f in
                let pelvis = f.joints.first { $0.id == GaitSignal.JointID.pelvis }!.worldPosition
                let joint = f.joints.first { $0.id == id }!.worldPosition
                return -(Double(joint.y) - Double(pelvis.y))
            }
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
            return 100 * coefficientOfVariation(kept.map { Double($0.1 - $0.0 + 1) })
        }
        return (cv(GaitSignal.JointID.ankle(.left)), cv(GaitSignal.JointID.ankle(.right)))
    }
}
