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
    /// **The gate against the pinned column is 0 of 3 clips, and that is a
    /// deliberate change from 1 of 3.** The pinned 16/44/6 column was measured
    /// with the retired ankle-height criterion AND with contact-duration scatter
    /// standing in for the runner's repeatability. Both inputs have since been
    /// replaced, so agreement would now be a coincidence rather than evidence —
    /// the control above (`testTheRetiredCriterionStillReproducesThePinned…`)
    /// is what proves the fixture is the one the column came from, and it still
    /// reproduces all three.
    ///
    /// What moved, and why:
    /// * `video_015` 11.144 → 8.086 %. Its repeatability input changed from
    ///   contact-duration CV (11.14 %) to STRIDE-PERIOD CV (2.56 %), so the
    ///   quantisation floor now binds — which also un-suppresses the camera
    ///   advice on the best of the three clips. See `GaitResolution`.
    /// * `video_013` 43.279 → 18.909 %, same cause (43.28 % → 18.91 %), plus its
    ///   frames-per-contact rose 4.4643 → 4.7024 once dropped frames stopped
    ///   shortening its contacts.
    /// * `video_012` unchanged at 10.145 %: its floor already dominated.
    func testG4ResolutionAgainstThePinnedColumn() throws {
        // `repeatability` is the RAW CV of the touchdown gaps; `published` is it
        // floored at what the clip could have distinguished
        // (`100/stridePeriodFrames`). `resolvable` is unchanged by that floor on
        // all three clips — see the assertion below, which is the point.
        let measured: [String: (framesPerContact: Double, floor: Double, repeatability: Double,
                                published: Double, resolvable: Double)] = [
            "video_012": (4.9286, 10.145, 0.000, 100.0 / 18.0, 10.145),
            "video_013": (4.7024, 10.633, 18.909, 18.909, 18.909),
            "video_015": (6.1833, 8.086, 2.564, 100.0 / 19.0, 8.086),
        ]
        for (clip, e) in measured {
            let r = try report(clip)
            XCTAssertEqual(r.resolution.framesPerContact, e.framesPerContact, accuracy: 0.01, clip)
            XCTAssertEqual(r.resolution.quantisationFloorPercent, e.floor, accuracy: 0.05, clip)
            XCTAssertEqual(r.resolution.measuredStrideRepeatabilityPercent, e.repeatability,
                           accuracy: 0.05, clip)
            XCTAssertEqual(r.resolution.strideRepeatabilityPercent, e.published,
                           accuracy: 0.05, clip)
            XCTAssertEqual(r.resolution.resolvableAsymmetryPercent, e.resolvable, accuracy: 0.05, clip)
            // The published number is never finer than the sampling grid allows.
            XCTAssertGreaterThanOrEqual(r.resolution.resolvableAsymmetryPercent,
                                        r.resolution.quantisationFloorPercent, clip)
            // And the repeatability INPUT is the stride period's scatter, not
            // the contact duration's — the two are measured separately and on
            // `video_015` they differ by 4.3×.
            XCTAssertEqual(r.resolution.measuredStrideRepeatabilityPercent,
                           largerFinite(r.strideVariationPercent.left,
                                        r.strideVariationPercent.right),
                           accuracy: 1e-9, clip)
            // The floor IS the steadiness bound, not a second constant that
            // could drift away from it.
            XCTAssertEqual(r.resolution.strideRepeatabilityBoundPercent,
                           r.steadiness.boundPercent, accuracy: 1e-12, clip)
            XCTAssertEqual(r.resolution.strideRepeatabilityPercent,
                           Swift.max(r.resolution.measuredStrideRepeatabilityPercent,
                                     r.resolution.strideRepeatabilityBoundPercent),
                           accuracy: 1e-12, clip)
        }
        let fifteen = try report("video_015")
        XCTAssertEqual(largerFinite(fifteen.contactVariationPercent.left,
                                    fifteen.contactVariationPercent.right),
                       11.144, accuracy: 0.05,
                       "the contact-duration scatter that used to be published as the runner's own")

        let pinned: [String: Double] = ["video_012": 16, "video_013": 44, "video_015": 6]
        var reproduced = 0
        for (clip, p) in pinned {
            let r = try report(clip)
            if abs(r.resolution.resolvableAsymmetryPercent - p) <= 1.0 { reproduced += 1 }
        }
        XCTAssertEqual(reproduced, 0, "0 of 3 clips reproduces the pinned resolution column")
    }

    func testTheQuantisationFloorIsHalfAFrameOverTheContact() throws {
        // The floor is arithmetic, not a fit: half a frame of edge uncertainty
        // spread over N frames of contact.
        for n in [4.0, 5.0, 6.0, 7.0, 8.3, 12.0] {
            let r = GaitResolution(framesPerContact: n, strideRepeatabilityPercent: 0, stridePeriodFrames: 0)
            XCTAssertEqual(r.quantisationFloorPercent, 100 * 0.5 / n, accuracy: 1e-9)
        }
        // The published figures for a 200 ms contact at the four capture rates,
        // which is the sentence the UI will show when it refuses a claim.
        for (fps, expected) in [(30.0, 8.3), (60.0, 4.2), (120.0, 2.1), (240.0, 1.0)] {
            let framesPerContact = 0.200 * fps
            let r = GaitResolution(framesPerContact: framesPerContact, strideRepeatabilityPercent: 0, stridePeriodFrames: 0)
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
                                    strideRepeatabilityPercent: 0,
                                    stridePeriodFrames: 0)
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

        // 4. Its resolution is 18.9 %, i.e. it could not have supported an
        //    asymmetry claim even if nothing else had failed.
        XCTAssertEqual(r.resolution.resolvableAsymmetryPercent, 18.909, accuracy: 0.05)

        // 5. And the dropped frames are now a REFUSAL, not just a flag: two of
        //    its contacts have a hole, so their duration is not resolved to
        //    ±½ a sampling interval and must not enter a left/right claim.
        let holes = r.refusals.compactMap { refusal -> (GaitSide, Int, Int)? in
            if case .droppedSamplesInContact(let s, let i, let e) = refusal { return (s, i, e) }
            return nil
        }
        XCTAssertFalse(holes.isEmpty, "a clip with holes in its contacts must be refused, not flagged")
        XCTAssertGreaterThan(holes.reduce(0) { $0 + $1.1 + $1.2 }, 0)

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

    // MARK: - What the periodicity check is, and what it is NOT

    /// The measured values, and the property they actually have.
    func testTheContactSequencePeriodicityCheckOnTheOwnersClips() throws {
        let expected: [String: (measuredMs: Double, modelledMs: Double, errorFrames: Double)] = [
            "video_012": (136.111, 135.714, 0.0119),
            "video_013": (121.212, 143.254, 0.6613),
            "video_015": (116.667, 116.389, 0.0083),
        ]
        for (clip, e) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.measuredFlightSeconds * 1000, e.measuredMs, accuracy: 0.5, clip)
            XCTAssertEqual(r.modelledFlightSeconds * 1000, e.modelledMs, accuracy: 0.5, clip)
            XCTAssertEqual(r.contactSequencePeriodicityErrorFrames, e.errorFrames,
                           accuracy: 0.02, clip)
        }
        XCTAssertLessThan(try report("video_012").contactSequencePeriodicityErrorFrames, 0.05)
        XCTAssertLessThan(try report("video_015").contactSequencePeriodicityErrorFrames, 0.05)
    }

    /// **The claim this file used to make, disproved here so it cannot be made
    /// again.**
    ///
    /// `NimbleEngine.GaitFrameOutcome` named this quantity as the check that
    /// covers the frame-level residual's admitted blindness to "the half-sine
    /// SHAPE and the peak magnitude". It cannot: on any perfectly periodic
    /// alternating schedule the two flight estimates are algebraically the SAME
    /// number, whatever the contact durations, and `Fmax` is a function of
    /// exactly those durations.
    ///
    /// Shown two ways: the algebra, exactly; and the shipped code path, where
    /// the quantity stays within 3 % of its gate while `Fmax` moves by a factor
    /// of 3.
    func testThePeriodicityCheckIsAnIdentityAndCannotSeeTheForce() throws {
        // 1. The algebra, independent of any detector. Touchdowns L at `nT` and
        //    R at `nT + s` give gaps `s − cL` and `T − s − cR`, whose mean is
        //    exactly `(T − cL − cR)/2` — the modelled flight — for every `s`,
        //    `cL` and `cR`. Nothing about the force enters either side.
        let stride = 0.60
        for s in [0.20, 0.30, 0.42] {
            for (cL, cR) in [(0.10, 0.10), (0.18, 0.09), (0.05, 0.25)] {
                let measured = 0.5 * ((s - cL) + (stride - s - cR))
                XCTAssertEqual(measured, 0.5 * (stride - cL - cR), accuracy: 1e-12,
                               "s=\(s) cL=\(cL) cR=\(cR)")
            }
        }

        // 2. The shipped path. Contact durations swept over a factor of 3 at a
        //    fixed stride, so `Fmax = (π/2)(1 + tf/tc)` sweeps with them.
        var peaks: [Double] = []
        var errors: [Double] = []
        for contact in [0.0667, 0.100, 0.150, 0.200] {
            let frames = Self.periodicSchedule(stride: stride, contact: contact, cycles: 6, fps: 300)
            let r = try GaitAnalysis.analyse(frames: frames)
            peaks.append(r.force.peakVerticalForceInBodyWeights)
            errors.append(r.contactSequencePeriodicityErrorFrames)
        }
        print("GAIT-METRIC periodicity_vs_force peaks=\(peaks) errors=\(errors)")
        // The NOMINAL schedules span a factor of 6 in Fmax; the detector's band
        // widens the short contacts, so what actually reaches the report spans
        // 1.63. Either way the force moves by tens of percent.
        let nominal = [0.0667, 0.200].map {
            GaitForceModel.peakInBodyWeights(contactSeconds: $0,
                                             flightSeconds: (stride - 2 * $0) / 2)
        }
        XCTAssertGreaterThan(nominal[0] / nominal[1], 2.9)
        XCTAssertGreaterThan(peaks.max()! / peaks.min()!, 1.5,
                             "Fmax moves by a factor of \(peaks.max()! / peaks.min()!)")
        XCTAssertLessThan(errors.max()!, 0.05 * GaitAnalysis.maximumDisagreementFrames,
                          "while the check never gets within 5 % of its own gate: \(errors)")
    }

    /// A synthetic runner whose contacts are placed on an exactly periodic,
    /// alternating schedule. Sampled fast enough (300 fps) that the detector
    /// resolves every duration in the sweep, so the identity above is not an
    /// artefact of quantisation.
    static func periodicSchedule(stride: Double, contact: Double, cycles: Int,
                                 fps: Double) -> [BodyFrame] {
        syntheticRunner(fps: fps, contact: contact, flight: (stride - 2 * contact) / 2,
                        speed: 4.0, strides: cycles, velocityNoise: 0.15)
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
                return TrackedJoint(id: j.id, name: j.name, worldPosition: frozen,
                                    isTracked: true,
                                    opensimMarkerNameOverride: j.opensimMarkerNameOverride)
            }
            edited.append(BodyFrame(timestamp: f.timestamp, frameNumber: f.frameNumber, joints: joints))
        }
        let r = try GaitAnalysis.analyse(frames: edited)
        XCTAssertFalse(r.isUsable, "a leg that misses a whole contact must not pass")
        XCTAssertFalse(r.refusals.isEmpty)
    }

    // MARK: - A dropped frame must not become a finding

    /// **The scenario, reproduced exactly.** Delete ONE frame from the middle of
    /// every LEFT contact of `video_015` — the decoder losing the person, which
    /// STATUS measures at 7.1 % of frames — and ask what the module reports.
    ///
    /// Counting surviving samples read the left contacts 205.6 → 144.4 ms
    /// against an unchanged right, i.e. −14.29 % of asymmetry against a 10.88 %
    /// resolution: a publishable finding, pointing the wrong way, made entirely
    /// of a decoder artefact. Timing off the clock reads the SAME duration it
    /// read before, and the hole is refused on top.
    func testAFrameLostInsideAContactChangesNoDurationAndRefusesTheClip() throws {
        let original = try frames("video_015")
        let clean = try GaitAnalysis.analyse(frames: original)
        XCTAssertTrue(clean.isUsable)

        // Which array positions sit in the middle of a left contact.
        let holes = Set(clean.stance.left.map { ($0.firstIndex + $0.lastIndex) / 2 })
        XCTAssertEqual(holes.count, clean.stance.left.count)
        let punched = original.enumerated()
            .filter { !holes.contains($0.offset) }
            .map(\.element)
        XCTAssertEqual(punched.count, original.count - holes.count)

        let r = try GaitAnalysis.analyse(frames: punched)

        // 1. Every left contact now has exactly one slot missing from inside
        //    it, and the clock still spans the whole contact: 5 samples over
        //    6 sampling intervals.
        for interval in r.stance.left {
            XCTAssertEqual(interval.droppedSamplesInside, 1)
            XCTAssertEqual(interval.seconds,
                           Double(interval.samples) * r.sampleInterval + r.sampleInterval,
                           accuracy: 1e-9,
                           "the clock covers the hole; counting samples does not")
        }
        for interval in r.stance.right {
            XCTAssertEqual(interval.droppedSamplesInside, 0)
        }

        // 2. The two conventions on the SAME analysis. Counting samples reads
        //    the left contact 14.29 % SHORTER than the right — above the 9.68 %
        //    floor, and with the sign reversed against what the clock says.
        let bySamples = Bilateral<Double>(
            left: mean(r.stance.left.map { Double($0.samples) }) * r.sampleInterval,
            right: mean(r.stance.right.map { Double($0.samples) }) * r.sampleInterval)
        let fabricated = 100 * (bySamples.left - bySamples.right)
            / (0.5 * (bySamples.left + bySamples.right))
        XCTAssertEqual(fabricated, -14.2857, accuracy: 0.01,
                       "the asymmetry the sample count manufactures")
        XCTAssertEqual(r.contactAsymmetryPercent, 6.4516, accuracy: 0.01,
                       "what the clock says instead — opposite sign")
        XCTAssertGreaterThan(abs(fabricated), r.resolution.resolvableAsymmetryPercent,
                             "and only the fabricated one clears the resolution gate")
        XCTAssertLessThan(abs(r.contactAsymmetryPercent), r.resolution.resolvableAsymmetryPercent)

        // 3. The clip is refused anyway, because a contact with a hole is not
        //    timed to ±½ a sampling interval whatever the clock says: a hole at
        //    an EDGE moves the retained edge inward by a real sampling interval
        //    and the clock cannot see that. Monte Carlo at the measured 7.1 %
        //    drop rate still reached 17-19 % of fabricated asymmetry with the
        //    clock fix alone, which is why this is a refusal and not just a
        //    corrected number.
        XCTAssertFalse(r.isUsable)
        XCTAssertNil(r.asymmetryClaim)
        XCTAssertTrue(r.refusals.contains {
            if case .droppedSamplesInContact(.left, _, _) = $0 { return true }; return false
        }, "\(r.refusals)")

        // And the clean clip has none of that, so the refusal discriminates.
        XCTAssertFalse(clean.refusals.contains {
            if case .droppedSamplesInContact = $0 { return true }; return false
        })
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
        // Sized from the MEDIAN contact, not the shortest. The shortest gives
        // 3 taps on `video_012`, whose second-derivative coefficients [1,−2,1]
        // amplify noise 21.5× against the 9-tap window and whose POSITION
        // coefficients are [0,1,0] — no smoothing at all.
        let expected: [String: (shortest: Int, median: Int, taps: Int)] = [
            "video_012": (4, 5, 5),
            "video_013": (1, 5, 5),
            "video_015": (5, 6, 5),
        ]
        for (clip, e) in expected {
            let r = try report(clip)
            XCTAssertEqual(r.shortestContactSamples, e.shortest, clip)
            XCTAssertEqual(r.medianContactSamples, e.median, clip)
            XCTAssertEqual(r.derivativeFilterTaps, e.taps, clip)
            XCTAssertFalse(r.nineTapFilterFitsOneContact, clip)
            XCTAssertEqual(r.nineTapFilterSpanSeconds, 8.0 / 30.0, accuracy: 1e-6, clip)
            // 4.69×, not 21.49×. The price is paid knowingly and published.
            XCTAssertEqual(WindowedDerivativeFilter
                .accelerationNoiseAmplification(taps: r.derivativeFilterTaps),
                           4.690, accuracy: 0.005, clip)
        }
        // At 120 fps a 200 ms contact spans 24 frames, and the 9-tap window
        // fits with room to spare — the same lever as the resolution.
        let fast = try GaitAnalysis.analyse(frames: Self.syntheticRunner(
            fps: 120, contact: 0.200, flight: 0.120, speed: 4.0, strides: 7,
            velocityNoise: 0.15))
        XCTAssertGreaterThanOrEqual(fast.derivativeFilterTaps, 9)
        XCTAssertTrue(fast.nineTapFilterFitsOneContact)
        XCTAssertEqual(WindowedDerivativeFilter
            .accelerationNoiseAmplification(taps: fast.derivativeFilterTaps),
                       1.0, accuracy: 1e-9,
                       "at a rate that resolves the contact there is no noise penalty at all")
    }

    /// **The per-leg peak is gated on the SAME resolution as the timing claim,
    /// and the impulse closes in both regimes.**
    ///
    /// One clip-wide `Fmax` applied to both feet unconditionally sets the timing
    /// model's own left/right peak asymmetry to zero by construction — the
    /// product exists to find that asymmetry — and it runs the wrong way round,
    /// since a shorter contact must carry a HIGHER peak.
    ///
    /// But a per-leg `Fmax` applied unconditionally is circular: the QP is linear
    /// in the external load, so each leg's peak lands inside every muscle's
    /// left/right comparison as a force scale. On `video_012` the contact
    /// difference is 2.899 % against a 10.145 % floor — the panel refuses it as
    /// unresolvable in one block and used to re-express it as −1.31 % of muscle
    /// asymmetry in the next. A number too noisy to display cannot be trusted to
    /// scale the comparison.
    ///
    /// So the previously pinned per-leg column (`video_012` 2.8499/2.8875,
    /// `video_015` 2.4602/2.4554) is DELIBERATELY replaced: both clips'
    /// asymmetries are below their own resolution, so both now read a single
    /// shared peak. The per-leg path is still exercised — by the resolvable
    /// schedule below, where it must survive.
    func testThePerLegPeakIsGatedOnTheSameResolutionAsTheTimingClaim() throws {
        // Both usable clips measure a contact asymmetry FAR below their own
        // resolution, so both collapse to one peak — and the value is the one
        // closed on the mean contact.
        let expectedShared: [String: Double] = ["video_012": 2.8686, "video_015": 2.4578]
        for (clip, shared) in expectedShared {
            let r = try report(clip)
            XCTAssertLessThan(abs(r.contactAsymmetryPercent),
                              r.resolution.resolvableAsymmetryPercent, clip)
            XCTAssertTrue(r.peakVerticalForceIsSharedBetweenLegs, clip)
            XCTAssertEqual(r.peakVerticalForceInBodyWeights.left,
                           r.peakVerticalForceInBodyWeights.right, accuracy: 0, clip)
            XCTAssertEqual(r.peakVerticalForceInBodyWeights.left, shared, accuracy: 0.005, clip)
            XCTAssertEqual(r.peakVerticalForceInBodyWeights.left,
                           GaitForceModel.peakInBodyWeights(
                               contactSeconds: 0.5 * (r.contactSeconds.left + r.contactSeconds.right),
                               flightSeconds: r.modelledFlightSeconds),
                           accuracy: 1e-12, clip)
            // And the stride's vertical impulse still closes exactly:
            // Σ Fᵢ·2·tcᵢ/π = T. This is the property the shared peak could have
            // broken and does not.
            let impulse = r.peakVerticalForceInBodyWeights.left * 2 * r.contactSeconds.left / .pi
                + r.peakVerticalForceInBodyWeights.right * 2 * r.contactSeconds.right / .pi
            let stride = r.contactSeconds.left + r.contactSeconds.right
                + 2 * r.modelledFlightSeconds
            XCTAssertEqual(impulse, stride, accuracy: 1e-9, "\(clip): m·g·T of impulse")
            print("GAIT-METRIC shared_peak clip=\(clip) "
                  + "contact_asymmetry_percent=\(r.contactAsymmetryPercent) "
                  + "resolvable_percent=\(r.resolution.resolvableAsymmetryPercent) "
                  + "peak_bw=\(r.peakVerticalForceInBodyWeights.left)")
        }

        // **The control the gate must not swallow.** 200 / 160 ms contacts with
        // 130 ms of flight is a 22 % contact asymmetry: resolvable at any
        // sensible floor, so each leg keeps its own peak and the shorter contact
        // carries the higher one.
        let asymmetric = GaitForceModel.perLegPeaksInBodyWeights(
            contactSeconds: Bilateral(left: 0.200, right: 0.160),
            flightSeconds: 0.130,
            resolvableAsymmetryPercent: 10.0)
        XCTAssertFalse(asymmetric.sharedBetweenLegs)
        XCTAssertEqual(asymmetric.peaks.left, 2.5918, accuracy: 0.001)
        XCTAssertEqual(asymmetric.peaks.right, 2.8471, accuracy: 0.001)
        XCTAssertLessThan(asymmetric.peaks.left, asymmetric.peaks.right,
                          "the shorter contact carries the higher peak")
        XCTAssertEqual(100 * (asymmetric.peaks.left - asymmetric.peaks.right)
                       / (0.5 * (asymmetric.peaks.left + asymmetric.peaks.right)),
                       -9.39, accuracy: 0.02,
                       "a 9.4 % peak asymmetry an unconditional shared model would discard")
        // The same contacts under a floor that refuses them: one peak, and the
        // impulse still closes.
        let refused = GaitForceModel.perLegPeaksInBodyWeights(
            contactSeconds: Bilateral(left: 0.200, right: 0.160),
            flightSeconds: 0.130,
            resolvableAsymmetryPercent: 30.0)
        XCTAssertTrue(refused.sharedBetweenLegs)
        XCTAssertEqual(refused.peaks.left, refused.peaks.right, accuracy: 0)
        XCTAssertEqual(refused.peaks.left,
                       GaitForceModel(contactSeconds: 0.180, flightSeconds: 0.130)
                           .peakVerticalForceInBodyWeights,
                       accuracy: 1e-12)
        XCTAssertEqual(refused.peaks.left * 2 * 0.200 / .pi + refused.peaks.right * 2 * 0.160 / .pi,
                       0.200 + 0.160 + 2 * 0.130, accuracy: 1e-12,
                       "the shared peak closes the stride impulse exactly too")
        XCTAssertEqual(refused.peaks.left, 2.7053, accuracy: 0.001)

        // Exactly at the boundary the difference IS claimable, so it is used.
        let boundary = GaitForceModel.perLegPeaksInBodyWeights(
            contactSeconds: Bilateral(left: 0.200, right: 0.160),
            flightSeconds: 0.130,
            resolvableAsymmetryPercent: 100 * 0.040 / 0.180)
        XCTAssertFalse(boundary.sharedBetweenLegs,
                       "the same inclusive boundary the asymmetry claim uses")
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
