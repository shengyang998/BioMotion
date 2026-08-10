import XCTest
import Combine
import simd
@testable import BioMotion

/// The dynamics path: root acceleration from the gait cycle instead of from
/// differentiating a depth channel, the ground force into inverse dynamics, and
/// the quantity that can contradict the model.
///
/// # Which falsifiability option was taken, and why
///
/// The choice was between **(a)** overriding the root acceleration and keeping
/// the near-CoP solver, and **(b)** applying the gait GRF as an external wrench
/// and running plain inverse dynamics so the ROOT residual becomes a genuine
/// force/moment residual.
///
/// **(a) for the mechanism, with three independent gates bolted on, because (b)
/// is not available on this pose source.** The measured reason: `MHRRetarget`
/// pins raw MHR joint 1 at the model constant, so `a_root` in the data is identically
/// zero and a plain-ID root residual would be `‖m·a_artic − m·g − F_gait‖ ≈
/// 3.9·m·g` on every stance frame of every clip, good and bad alike. A quantity
/// that reports the same failure on all inputs is a constant, not a falsifier.
/// (b) becomes available once `cam_t` is composed in AND its depth channel is
/// usable; STATUS measures 3.11 g of pure noise there at 30 fps.
///
/// So the falsification burden is carried by three quantities that CAN each
/// disagree, and that gate the output together (`GaitLoadSummary.arePublishable`):
///
/// 1. `‖ΣF_contact − F_gait‖/(m·g)` = `‖a_artic‖/g`, over the frames where both
///    contact detectors agree. `testTheGateFiresWhenTheOmittedTermIsLarge`
///    shows it firing.
/// 2. The ID solver's own GEOMETRIC contact detector against the KINEMATIC
///    stance detector — two different signals, foot height versus pelvis-
///    relative horizontal velocity.
/// 3. Per-muscle saturation, which is exactly where the QP stops being linear
///    in the external load and a peak-force error stops cancelling out of a
///    ratio.
///
/// ⚠️ Stated plainly because it bounds the claim: **nothing here tests the peak
/// magnitude or the half-sine shape.** Both enter `a_root` and cancel out of
/// (1); `GaitReport.contactSequencePeriodicityErrorFrames` is an algebraic
/// identity on any periodic alternating schedule and cannot see them either
/// (proved in `GaitReportTests`). What licenses shipping anyway is (3): a
/// peak-force error is a common scale, and gate (3) is the condition under
/// which a common scale cancels.
final class GaitDynamicsTests: XCTestCase {

    private static let g = StaticHoldDetector.gravityMetersPerSecondSquared

    // MARK: - The mechanism, measured on the real skeleton

    /// **The load-bearing measurement of this whole stage.** Writing an
    /// acceleration into the root's `pelvis_ty` coordinate must produce a
    /// WORLD-VERTICAL inertial force of exactly `m·a` — otherwise the gait
    /// model's root acceleration is being injected into some body-local axis
    /// and every torque downstream is wrong in a way nothing would flag.
    ///
    /// Measured directly: plain inverse dynamics at rest reports the root's
    /// generalized force; adding `a` to `ddq[pelvis_ty]` must move it by
    /// `m·a` and nothing else.
    func testWritingRootVerticalAccelerationProducesExactlyMassTimesAcceleration() throws {
        let bridge = try loadedBridge()
        let names = bridge.dofNames
        let ty = try XCTUnwrap(names.firstIndex(of: NimbleEngine.rootVerticalDOFName),
                               "the shipped model must carry \(NimbleEngine.rootVerticalDOFName)")
        let n = names.count
        let mass = bridge.totalMass
        XCTAssertGreaterThan(mass, 20, "a person's mass")

        let zeros = [NSNumber](repeating: 0, count: n)
        let rest = try XCTUnwrap(
            bridge.solveUnvalidatedIDForDiagnostics(
                withJointAngles: zeros,
                jointVelocities: zeros,
                jointAccelerations: zeros),
            "the zero-external-force ID diagnostic must solve at the neutral pose")
        let restForce = rest.jointTorques[ty].doubleValue

        for a in [1.0, -Self.g, 2.0 * Self.g] {
            var ddq = [NSNumber](repeating: 0, count: n)
            ddq[ty] = NSNumber(value: a)
            let moved = try XCTUnwrap(
                bridge.solveUnvalidatedIDForDiagnostics(
                    withJointAngles: zeros,
                    jointVelocities: zeros,
                    jointAccelerations: ddq))
            let delta = moved.jointTorques[ty].doubleValue - restForce
            print("GAIT-METRIC root_ty a=\(a) delta_N=\(delta) expected=\(mass * a) mass=\(mass)")
            XCTAssertEqual(delta, mass * a, accuracy: max(1e-6, abs(mass * a) * 1e-9),
                           "pelvis_ty must be a world-vertical translation of the whole body")
        }

        // And it must be VERTICAL: the same acceleration written into `ty` must
        // not move the fore-aft or lateral root force.
        let tx = try XCTUnwrap(names.firstIndex(of: "pelvis_tx"))
        let tz = try XCTUnwrap(names.firstIndex(of: "pelvis_tz"))
        var ddq = [NSNumber](repeating: 0, count: n)
        ddq[ty] = NSNumber(value: 5.0)
        let moved = try XCTUnwrap(
            bridge.solveUnvalidatedIDForDiagnostics(
                withJointAngles: zeros,
                jointVelocities: zeros,
                jointAccelerations: ddq))
        XCTAssertEqual(moved.jointTorques[tx].doubleValue,
                       rest.jointTorques[tx].doubleValue, accuracy: 1e-6,
                       "a vertical root acceleration must not appear in the fore-aft channel")
        XCTAssertEqual(moved.jointTorques[tz].doubleValue,
                       rest.jointTorques[tz].doubleValue, accuracy: 1e-6)
    }

    /// The value actually written: `a = g·(F/mg − 1)`. Flight must be exactly
    /// free fall, and that is the step the whole route rests on.
    func testFlightIsFreeFallAndPeakStanceIsPhysiological() {
        func rootAccel(_ bw: Double) -> Double { Self.g * (bw - 1) }
        XCTAssertEqual(rootAccel(0), -Self.g, accuracy: 1e-12, "flight is free fall, exactly")
        XCTAssertEqual(rootAccel(1), 0, accuracy: 1e-12, "one body weight is equilibrium")
        // 2.87 BW is `video_012`'s modelled peak.
        XCTAssertEqual(rootAccel(2.8684) / Self.g, 1.8684, accuracy: 1e-3)
    }

    // MARK: - The plan, built from the owner's own clips

    /// The plan must be the derivation, not a shape that merely looks like it:
    /// over one stride the mean vertical force has to be exactly one body
    /// weight, because that is the impulse closure `∫F dt = m·g·T` the peak was
    /// derived from.
    func testThePlanClosesTheStrideImpulseOnEveryUsableClip() throws {
        var analysed = 0
        for id in GaitClipFixture.allIds {
            let frames = try GaitClipFixture.load(id, bundle: Bundle(for: type(of: self))).frames
            let report = try GaitAnalysis.analyse(frames: frames)
            guard report.isUsable else {
                print("GAIT-METRIC clip=\(id) refused: \(report.refusals.map(\.description))")
                continue
            }
            analysed += 1
            let plan = try XCTUnwrap(OfflineSessionRunner.makePlan(from: report))

            let stance = plan.frames.filter { $0.contactSide != 0 }
            let flight = plan.frames.filter { $0.contactSide == 0 }
            XCTAssertFalse(stance.isEmpty)
            for f in flight {
                XCTAssertEqual(f.verticalForceInBodyWeights, 0, "flight carries no ground force")
            }
            // The plan's peak is the LARGER of the two legs' own peaks, not the
            // clip mean: each contact is closed on its own contact time, so the
            // shorter contact carries a higher peak than the mean model does.
            let peak = stance.map(\.verticalForceInBodyWeights).max() ?? 0
            let perLegMax = Swift.max(report.peakVerticalForceInBodyWeights.left,
                                      report.peakVerticalForceInBodyWeights.right)
            XCTAssertEqual(peak, perLegMax, accuracy: 1e-9)
            XCTAssertGreaterThanOrEqual(perLegMax,
                                        report.force.peakVerticalForceInBodyWeights - 1e-9,
                                        "\(id): the per-leg peak brackets the clip mean")

            // The closure. Mean force over the whole covered span, in BW.
            let mean = plan.frames.map(\.verticalForceInBodyWeights).reduce(0, +)
                / Double(plan.frames.count)
            print("GAIT-METRIC clip=\(id) plan_frames=\(plan.frames.count) "
                  + "stance=\(stance.count) flight=\(flight.count) taps=\(plan.filterTaps) "
                  + "peak_bw=\(peak) mean_bw=\(mean) "
                  + "model_peak_bw=\(report.force.peakVerticalForceInBodyWeights)")
            XCTAssertEqual(mean, 1.0, accuracy: 0.25,
                           "\(id): a steady stride's mean vertical force is one body weight")
        }
        XCTAssertGreaterThanOrEqual(analysed, 2, "two of the three pinned clips are usable")
    }

    /// A refused clip must produce no plan-driven dynamics at all. `video_013`
    /// is the registered failure (stride variation 18.9 % against a 5.6 % bound)
    /// and it must stay refused.
    func testTheRefusedClipYieldsNoDynamics() throws {
        let frames = try GaitClipFixture.load("video_013", bundle: Bundle(for: type(of: self))).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        XCTAssertFalse(report.isUsable, "video_013 is the registered refusal")
        XCTAssertFalse(report.refusals.isEmpty)
        print("GAIT-METRIC video_013_refusals=\(report.refusals.map(\.description))")
    }

    /// Each stance sample sits at the midpoint of its interval, so no sample
    /// claims exactly zero force at an instant the foot is measurably planted,
    /// and the profile is symmetric about mid-stance.
    func testStanceForceIsAHalfSineSampledAtIntervalMidpoints() throws {
        let frames = try GaitClipFixture.load("video_015", bundle: Bundle(for: type(of: self))).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        let plan = try XCTUnwrap(OfflineSessionRunner.makePlan(from: report))
        let interval = try XCTUnwrap(report.stance.left.first)

        var forces: [Double] = []
        for (k, t) in interval.sampleTimestamps.enumerated() {
            _ = k
            let entry = try XCTUnwrap(plan.entry(at: t), "no plan entry at stance sample \(k)")
            XCTAssertEqual(entry.contactSide, -1, "left contact")
            forces.append(entry.verticalForceInBodyWeights)
        }
        XCTAssertGreaterThan(forces.first!, 0, "a planted foot never carries exactly zero")
        XCTAssertEqual(forces.first!, forces.last!, accuracy: 1e-9, "symmetric about mid-stance")
        let peak = forces.max()!
        // The LEFT leg's own peak, not the clip mean — each contact is closed
        // on its own contact time.
        let leftPeak = report.peakVerticalForceInBodyWeights.left
        XCTAssertEqual(peak, leftPeak, accuracy: 0.05 * leftPeak)
        print("GAIT-METRIC halfsine_profile=\(forces.map { String(format: "%.3f", $0) })")
    }

    /// A timestamp between planned samples, or outside the covered span, must
    /// return nothing rather than the nearest guess.
    func testThePlanRefusesInstantsItDoesNotCover() {
        let dt = 1.0 / 30.0
        let plan = NimbleEngine.GaitPlan(
            frames: [.init(timestamp: 1.0, verticalForceInBodyWeights: 2.0, contactSide: 1,
                           contactIndex: 0, derivativeWindowInsideContact: true),
                     .init(timestamp: 1.0 + dt, verticalForceInBodyWeights: 1.0, contactSide: 1,
                           contactIndex: 0, derivativeWindowInsideContact: true)],
            filterTaps: 5, sampleInterval: dt)
        XCTAssertNotNil(plan.entry(at: 1.0))
        XCTAssertNotNil(plan.entry(at: 1.0 + dt * 0.4))
        XCTAssertNil(plan.entry(at: 0.5), "before the covered span")
        XCTAssertNil(plan.entry(at: 5.0), "after it")
        XCTAssertNil(plan.entry(at: 1.0 - dt), "one whole frame early")
    }

    // MARK: - End to end through the engine

    /// A gait plan remains useful for kinematic stance/flight timing even when
    /// this model/solver pair cannot justify contact-dependent loads. Stance
    /// must retain its gait label while publishing no ID, muscle, or residual;
    /// flight/outside-policy reasons still take precedence over capability.
    @MainActor
    func testRunningSequenceRetainsTimingButPublishesNoContactDynamics() async throws {
        let engine = try await loadedEngine()
        let dt = 1.0 / 30.0
        let taps = 5
        let sequence = Self.syntheticRunSequence(dt: dt, frames: 24, swingAmplitude: 0.03)
        let plan = Self.plan(for: sequence, dt: dt, taps: taps, peakBW: 2.5,
                             contactFrames: 7)

        engine.staticHoldGating = false
        engine.gaitPlan = plan

        var stance = 0, flight = 0, outside = 0, unconverged = 0

        for (i, markers) in sequence.enumerated() {
            let ok = await submitAndWait(engine, bodyFrame(markers, timestamp: Double(i) * dt,
                                                           frameNumber: i))
            guard ok, let solve = engine.lastSolve else { continue }
            switch solve.motion.verdict {
            case .gaitStance:
                stance += 1
                XCTAssertEqual(solve.dynamicsAvailability, .contactSupportUnavailable)
                XCTAssertNil(solve.id)
                XCTAssertNil(solve.muscle)
                XCTAssertNil(solve.gait,
                             "a timing stance is not a measured gait-load outcome")
            case .gaitFlight:
                flight += 1
                XCTAssertEqual(solve.dynamicsAvailability, .withheld(.gaitFlight))
                XCTAssertNil(solve.id)
                XCTAssertNil(solve.muscle)
                XCTAssertNil(solve.gait)
            case .gaitOutsideAnalysis:
                outside += 1
                XCTAssertEqual(solve.dynamicsAvailability, .withheld(.gaitOutsideAnalysis))
            case .poseDidNotConverge:
                unconverged += 1
                XCTAssertEqual(solve.dynamicsAvailability, .withheld(.poseDidNotConverge))
                XCTAssertNil(solve.id)
                XCTAssertNil(solve.muscle)
            default:
                XCTFail("a clip with a gait plan must not report \(solve.motion.verdict)")
            }
        }
        engine.gaitPlan = nil

        print("GAIT-METRIC pose_only engine stance=\(stance) flight=\(flight) "
              + "outside=\(outside) unconverged=\(unconverged)")

        XCTAssertGreaterThan(stance, 0, "the plan's contacts must reach the engine")
        XCTAssertGreaterThan(flight, 0, "and so must its flight phases")
        XCTAssertNil(engine.lastIDResult)
        XCTAssertNil(engine.lastMuscleResult)
    }

    /// The residual gate remains the contract for a future input whose contact
    /// support has independently been validated. Synthetic summaries may test
    /// that algorithm; the bundled engine must never fabricate the inputs.
    func testResidualGateRejectsAValidatedSyntheticInputAboveItsBound() throws {
        let refused = Self.summary(maxResidual: 1.78)
        XCTAssertFalse(refused.residualGatePassed)
        XCTAssertFalse(refused.clearsStatisticalFloor(Self.load(left: 0.9, right: 0.1)))
        XCTAssertTrue(try XCTUnwrap(refused.withheldReason).contains("Withheld"))
    }

    // MARK: - The static-hold classification remains independent

    /// A genuinely motionless subject remains a hold before and after a gait
    /// pass. What must not survive either boundary is an unconstrained contact
    /// solve or any older ID/muscle/gait-load payload.
    @MainActor
    func testAMotionlessSubjectStillHoldsBeforeAndAfterAGaitPassWithoutDynamics() async throws {
        let engine = try await loadedEngine()

        func holdSolve() async throws -> NimbleEngine.SolveRecord {
            engine.resetSessionState()
            engine.setExplicitGroundHeightY(Self.trustedFixtureGroundY)
            engine.gaitPlan = nil
            engine.staticHoldGating = true
            let dt = 0.5
            for push in 0..<SavitzkyGolayFilter.windowSize {
                let ts = Double(push - SavitzkyGolayFilter.halfWindow) * dt
                _ = await submitAndWait(engine, bodyFrame(Self.dancerMarkers,
                                                          timestamp: ts, frameNumber: push))
            }
            return try XCTUnwrap(engine.lastSolve, "a full window must publish a solve")
        }

        func assertIsPoseOnlyHold(_ s: NimbleEngine.SolveRecord, _ label: String) {
            XCTAssertTrue(s.motion.isHold, "\(label): nine replays of one pose is a hold")
            XCTAssertEqual(s.motion.verdict, .hold, label)
            XCTAssertFalse(s.motion.verdict.isGait, label)
            XCTAssertEqual(s.dynamicsAvailability, .contactSupportUnavailable, label)
            XCTAssertFalse(s.isStaticHoldEstimate, label)
            XCTAssertNil(s.muscle, label)
            XCTAssertNil(s.id, label)
            XCTAssertNil(s.gait, "\(label): a hold carries no gait outcome")
            XCTAssertEqual(s.centerTimestamp, 0.0, accuracy: 1e-6, label)
        }

        let a = try await holdSolve()
        assertIsPoseOnlyHold(a, "first hold")
        let b = try await holdSolve()
        assertIsPoseOnlyHold(b, "repeat hold")

        // A full gait pass on the same engine must not change the later hold
        // classification or leave an unavailable dynamics payload behind.
        let dt = 1.0 / 30.0
        let sequence = Self.syntheticRunSequence(dt: dt, frames: 20, swingAmplitude: 0.03)
        engine.resetSessionState()
        engine.setExplicitGroundHeightY(Self.trustedFixtureGroundY)
        engine.staticHoldGating = false
        engine.gaitPlan = Self.plan(for: sequence, dt: dt, taps: 5, peakBW: 2.5)
        for (i, markers) in sequence.enumerated() {
            _ = await submitAndWait(engine, bodyFrame(markers, timestamp: Double(i) * dt,
                                                      frameNumber: i))
        }
        engine.gaitPlan = nil

        let c = try await holdSolve()
        assertIsPoseOnlyHold(c, "hold after a gait pass")
        XCTAssertNil(engine.lastIDResult)
        XCTAssertNil(engine.lastMuscleResult)
        XCTAssertNil(engine.displayMuscleResult)
    }

    // MARK: - Reproducibility across a clip boundary

    /// Import clip B, then A, then B again. The same clip must retain identical
    /// kinematic routing while every stance remains contact-unavailable; a
    /// preceding clip must not resurrect an older load payload.
    @MainActor
    func testRepeatedRunsRemainDeterministicallyPoseOnly() async throws {
        let engine = try await loadedEngine()
        let dt = 1.0 / 30.0
        let clipA = Self.syntheticRunSequence(dt: dt, frames: 20, swingAmplitude: 0.14)
        let clipB = Self.syntheticRunSequence(dt: dt, frames: 20, swingAmplitude: 0.03)

        func run(_ sequence: [[(String, SIMD3<Double>)]]) async
            -> [(TimeInterval, NimbleEngine.MotionVerdict,
                 NimbleEngine.DynamicsAvailability)] {
            engine.resetSessionState()
            engine.setExplicitGroundHeightY(Self.trustedFixtureGroundY)
            engine.staticHoldGating = false
            engine.gaitPlan = Self.plan(for: sequence, dt: dt, taps: 5, peakBW: 2.5)
            var out: [(TimeInterval, NimbleEngine.MotionVerdict,
                       NimbleEngine.DynamicsAvailability)] = []
            for (i, markers) in sequence.enumerated() {
                let ok = await submitAndWait(engine, bodyFrame(markers, timestamp: Double(i) * dt,
                                                               frameNumber: i))
                guard ok, let solve = engine.lastSolve else { continue }
                XCTAssertNil(solve.id)
                XCTAssertNil(solve.muscle)
                XCTAssertNil(solve.gait)
                out.append((solve.centerTimestamp, solve.motion.verdict,
                            solve.dynamicsAvailability))
            }
            engine.gaitPlan = nil
            return out
        }

        let alone = await run(clipB)
        _ = await run(clipA)
        let afterA = await run(clipB)

        XCTAssertFalse(alone.isEmpty, "clip B must publish centred poses")
        XCTAssertEqual(alone.count, afterA.count,
                       "same clip, same number of solved frames (a mismatch here is the test "
                       + "harness dropping a submission, not the solver)")
        for (x, y) in zip(alone, afterA) {
            XCTAssertEqual(x.0, y.0, accuracy: 1e-12, "frames must line up in time")
            XCTAssertEqual(x.1, y.1)
            XCTAssertEqual(x.2, y.2)
            if x.1 == .gaitStance {
                XCTAssertEqual(x.2, .contactSupportUnavailable)
            }
        }
        XCTAssertNil(engine.lastIDResult)
        XCTAssertNil(engine.lastMuscleResult)
    }

    /// With no plan the engine uses the window that shipped, so the live camera
    /// path is on the same code and the same 9 taps as before.
    func testWithoutAPlanTheWindowIsTheOneThatShipped() {
        XCTAssertEqual(WindowedDerivativeFilter.maximumTaps, SavitzkyGolayFilter.windowSize)
        let f = WindowedDerivativeFilter()
        XCTAssertEqual(f.taps, SavitzkyGolayFilter.windowSize)
        XCTAssertEqual(f.halfWindow, SavitzkyGolayFilter.halfWindow)
        XCTAssertEqual(f.order, 3)
    }

    // MARK: - Fixtures

    private static var dancerMarkers: [(String, SIMD3<Double>)] {
        OfflineMuscleChainFixture.markers.map { _, opensim, p in
            (opensim, SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z)))
        }
    }

    /// The FullBody model's world-ground convention. These tests isolate gait
    /// dynamics from rolling-floor calibration, which has its own integration
    /// contracts in `OfflineOrchestrationTests`.
    private static let trustedFixtureGroundY = 0.0

    /// A running-like 20-marker sequence: the whole body bounces vertically at
    /// stride frequency while the limb markers swing at twice that. Contacts
    /// are the bottom of the bounce. Synthetic on purpose — the pinned clip
    /// fixture carries only 5 of the 20 markers the solver needs, so it cannot
    /// drive a whole-body inverse-dynamics solve.
    private static func syntheticRunSequence(dt: Double, frames: Int,
                                             swingAmplitude: Double) -> [[(String, SIMD3<Double>)]] {
        let strideHz = 1.65     // 606 ms stride, the owner's own cadence
        let base = dancerMarkers
        let limbs: Set<String> = ["LKJC", "RKJC", "LAJC", "RAJC", "LTOE", "RTOE",
                                  "LEJC", "REJC", "LWJC", "RWJC"]
        return (0..<frames).map { i in
            let t = Double(i) * dt
            let bounce = 0.04 * sin(2 * .pi * strideHz * t)
            return base.map { name, p in
                var q = p
                q.y += bounce
                if limbs.contains(name) {
                    let phase = name.hasPrefix("L") ? 0.0 : Double.pi
                    q.x += swingAmplitude * sin(2 * .pi * 2 * strideHz * t + phase)
                    q.y += swingAmplitude * 0.5 * cos(2 * .pi * 2 * strideHz * t + phase)
                }
                return (name, q)
            }
        }
    }

    /// Alternating contacts: 5 frames left, 4 flight, 5 right, 4 flight, …
    private static func plan(for sequence: [[(String, SIMD3<Double>)]],
                             dt: Double, taps: Int, peakBW: Double,
                             contactFrames: Int = 5) -> NimbleEngine.GaitPlan {
        var entries: [NimbleEngine.GaitPlan.Frame] = []
        let contact = contactFrames, flight = 4
        let cycle = contact + flight
        // One index per foot-strike, so the summary groups by the plan's own
        // boundaries rather than by which frames happened to arrive.
        var contactIndex = -1
        var wasInContact = false
        for i in sequence.indices {
            let t = Double(i) * dt
            let phaseIndex = i % (2 * cycle)
            let inFirst = phaseIndex < contact
            let inSecond = phaseIndex >= cycle && phaseIndex < cycle + contact
            if inFirst || inSecond {
                let k = inFirst ? phaseIndex : phaseIndex - cycle
                if !wasInContact { contactIndex += 1 }
                wasInContact = true
                let phase = (Double(k) + 0.5) / Double(contact)
                entries.append(.init(timestamp: t,
                                     verticalForceInBodyWeights: peakBW * sin(.pi * phase),
                                     contactSide: inFirst ? -1 : 1,
                                     contactIndex: contactIndex,
                                     derivativeWindowInsideContact:
                                        k >= taps / 2 && k <= contact - 1 - taps / 2))
            } else {
                wasInContact = false
                entries.append(.init(timestamp: t, verticalForceInBodyWeights: 0, contactSide: 0,
                                     contactIndex: -1,
                                     derivativeWindowInsideContact: false))
            }
        }
        return NimbleEngine.GaitPlan(frames: entries, filterTaps: taps, sampleInterval: dt)
    }

    private static func load(left: Double, right: Double) -> GaitLoadSummary.MuscleLoad {
        .init(id: "glmax1", displayName: "Glute max (upper)",
              leftLoad: left, rightLoad: right, leftContacts: 5, rightContacts: 5,
              isSaturated: false, isAtActivationFloor: false,
              samplingUncertaintyPercent: 0, pathIsModelled: false)
    }

    private static func summary(maxResidual: Double) -> GaitLoadSummary {
        GaitLoadSummary(muscles: [load(left: 0.9, right: 0.1)],
                        resolvableAsymmetryPercent: 10,
                        quantisationFloorPercent: 10,
                        strideRepeatabilityPercent: 7,
                        measuredStrideRepeatabilityPercent: 7,
                        strideRepeatabilityBoundPercent: 5.56,
                        contactClaimFloorPercent: 10,
                        contactSamplingUncertaintyPercent: 0,
                        peakForceIsSharedBetweenLegs: true,
                        contactTimeContributionPercent: 0,
                        framesPerContact: 5,
                        framesPerSecond: 30,
                        stanceFrameCount: 10,
                        claimedStanceFrameCount: 10,
                        screenedComparisonCount: 1,
                        saturatedMuscleCount: 0,
                        flooredMuscleCount: 0,
                        maxVerticalForceResidualInBodyWeights: maxResidual,
                        medianVerticalForceResidualInBodyWeights: maxResidual,
                        residualFrameCount: 10,
                        residualGatePassed: maxResidual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
                        contactDetectorDisagreements: 0,
                        solverSawDoubleContactCount: 0,
                        framesWithoutACleanDerivativeWindow: 0,
                        leftStanceFrameCount: 5,
                        rightStanceFrameCount: 5,
                        leftContactCount: 5,
                        rightContactCount: 5,
                        horizontalRootAccelerationModelled: false,
                        derivativeFilterTaps: 5,
                        derivativeFilterSpanMilliseconds: 133,
                        shortestContactMilliseconds: 167,
                        derivativeNoiseAmplification: 4.69)
    }

    private func bodyFrame(_ markers: [(String, SIMD3<Double>)],
                           timestamp: TimeInterval, frameNumber: Int) -> BodyFrame {
        let joints: [TrackedJoint] = markers.compactMap { opensim, p in
            guard let m = JointMapping.primary.first(where: {
                $0.opensimName == opensim || (opensim == "MHR_ROOT" && $0.arkitName == "hips_joint")
            }) else { return nil }
            return TrackedJoint(id: m.arkitName, name: m.displayName,
                                worldPosition: SIMD3<Float>(Float(p.x), Float(p.y), Float(p.z)),
                                isTracked: true,
                                opensimMarkerNameOverride: opensim)
        }
        return BodyFrame(timestamp: timestamp, frameNumber: frameNumber, joints: joints)
    }

    private func loadedBridge() throws -> NimbleBridge {
        let bridge = NimbleBridge()
        let bundle = Bundle(for: type(of: self))
        let path = bundle.path(forResource: "FullBody", ofType: "osim")
            ?? Bundle.main.path(forResource: "FullBody", ofType: "osim")
            ?? bundle.path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        let resolved = try XCTUnwrap(path, "no .osim in the bundle")
        XCTAssertTrue(bridge.loadModel(fromPath: resolved))
        return bridge
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
        // These tests exercise the gait substitution/falsifier, not floor
        // estimation. Their contact fixture therefore declares its ground.
        engine.setExplicitGroundHeightY(Self.trustedFixtureGroundY)
        return engine
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
}
