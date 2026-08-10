import XCTest
import UIKit
@testable import BioMotion

/// **What the screen claims, per frame and per clip.**
///
/// Two defects live here, and they are the same defect twice: a number the
/// pipeline had already refused reaching the user under a caption that says it
/// was measured.
///
/// 1. The 3-D muscle overlay and the frame caption were gated CLIP-wide only, so
///    a clip that passed every gate still drew coloured muscle capsules on the
///    individual frames `GaitLoadSummary.make` had discarded.
/// 2. The truncation banner named the wrong cause and the wrong frame count for
///    the case it most often fires on.
final class OfflineDisclosureTests: XCTestCase {

    // MARK: - The per-frame gate on the overlay and the caption

    /// A frame whose derivative window crossed a contact edge does not enter the
    /// ranked list, and now does not draw either. MEASURED share of stance
    /// frames this affects on the pinned clips: 52 of 64 (`video_012`) and 44 of
    /// 68 (`video_015`).
    func testAFrameOutsideTheLoadComparisonDrawsNoMuscleOverlay() {
        let clean = Self.stanceFrame(side: -1, cleanWindow: true)
        XCTAssertTrue(clean.gaitLoadsAreComparable)
        XCTAssertNil(clean.gaitExclusionReason)

        let dirty = Self.stanceFrame(side: -1, cleanWindow: false)
        XCTAssertFalse(dirty.gaitLoadsAreComparable,
                       "the ranked list drops this frame; so must the overlay")
        XCTAssertEqual(dirty.gaitExclusionReason, "too close to a touchdown or toe-off "
                       + "to differentiate")
        // It still carries muscle numbers — the ONLY thing between them and the
        // screen is this flag, which is exactly why it has to be asked.
        XCTAssertNotNil(dirty.muscleResult)
    }

    /// Each exclusion names its own lever. Collapsing them would tell a user
    /// with a ground-height problem to film at a higher frame rate.
    func testEachExclusionNamesItsOwnCause() {
        let noFootDown = Self.stanceFrame(side: -1, solverLeft: false, solverRight: false)
        XCTAssertFalse(noFootDown.gaitLoadsAreComparable)
        XCTAssertEqual(noFootDown.gaitExclusionReason,
                       "the foot's height above the ground disagrees that it was planted — "
                       + "solved with no ground force")

        let doubleContact = Self.stanceFrame(side: -1, solverLeft: true, solverRight: true)
        XCTAssertFalse(doubleContact.gaitLoadsAreComparable)
        XCTAssertEqual(doubleContact.gaitExclusionReason,
                       "the solver put ground force under BOTH feet, so this frame's load is "
                       + "split between them")

        // A flight frame carrying an outcome does not occur today —
        // `NimbleEngine` routes flight through `publishPoseOnly`, so it has no
        // outcome and no muscle result. This pins the defensive branch so that
        // if that ever changes the sentence is the right one rather than
        // "too close to a touchdown".
        let flight = Self.gaitFrame(contactSide: 0, solverLeft: false, solverRight: false,
                                    cleanWindow: false)
        XCTAssertFalse(flight.gaitLoadsAreComparable)
        XCTAssertEqual(flight.gaitExclusionReason, "both feet off the ground — no contact load here")
    }

    /// **The non-regression that licenses the change.** A still-pose clip has no
    /// gait outcome at all, so nothing about the static-hold path moves.
    func testTheStillPosePathIsUnchanged() {
        let hold = OfflineResultStore.FrameResult(
            id: 0, sourceImage: UIImage(), timestamp: 0, status: .success,
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil,
            muscleResult: NimbleEngine.MuscleOutput(activations: ["soleus_l": 0.4],
                                                    forces: ["soleus_l": 400],
                                                    converged: true, timestamp: 0),
            isStaticHoldEstimate: true,
            motionState: .measured(verdict: .hold, peakSpeedMetersPerSecond: 0.001,
                                   windowSeconds: 1.0, noiseFloorMetersPerSecond: 0.004))
        XCTAssertTrue(hold.gaitLoadsAreComparable)
        XCTAssertNil(hold.gaitExclusionReason)

        let undetermined = OfflineResultStore.FrameResult(
            id: 1, sourceImage: UIImage(), timestamp: 0, status: .success,
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false,
            motionState: .undetermined)
        XCTAssertTrue(undetermined.gaitLoadsAreComparable)
    }

    /// The per-frame flag and the clip-level statistic have to agree about which
    /// frames they are talking about, or the panel's count and the overlay's
    /// behaviour drift apart.
    func testThePerFrameFlagAgreesWithWhatTheSummaryCounted() throws {
        let bundle = Bundle(for: type(of: self))
        let report = try GaitAnalysis.analyse(
            frames: try GaitClipFixture.load("video_012", bundle: bundle).frames)
        var frames: [OfflineResultStore.FrameResult] = []
        for i in 0..<8 {
            frames.append(Self.stanceFrame(id: i, side: i < 4 ? -1 : 1,
                                           cleanWindow: i % 2 == 0))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.stanceFrameCount, frames.filter(\.gaitLoadsAreComparable).count)
        XCTAssertEqual(s.claimedStanceFrameCount - s.stanceFrameCount,
                       frames.filter { !$0.gaitLoadsAreComparable }.count)
    }

    /// Consumer-side fail-closed guard: even a contradictory payload carrying
    /// an old gait outcome must not smuggle a residual through after the same
    /// frame says its dynamics are unavailable.
    func testUnavailableDynamicsCannotEnterTheGaitResidualSummary() throws {
        let report = try GaitAnalysis.analyse(
            frames: try GaitClipFixture.load(
                "video_012", bundle: Bundle(for: type(of: self))).frames)
        let contradictory = Self.gaitFrame(
            contactSide: -1,
            solverLeft: true,
            solverRight: false,
            cleanWindow: true,
            dynamicsAvailability: .groundPlaneUntrusted)

        XCTAssertFalse(contradictory.gaitLoadsAreComparable)
        XCTAssertNil(
            GaitLoadSummary.make(frames: [contradictory], report: report, filterTaps: 5),
            "an unavailable frame must not contribute even a residual-only summary"
        )
    }

    /// Detection provenance is not itself a failure: a still photo may use the
    /// whole image and remain analysable. The same event inside a video is a
    /// temporal discontinuity, so its pose stays reviewable but every solve
    /// field is fail-closed both at admission and at the result-store seam.
    @MainActor
    func testVideoFallbackIsPoseOnlyWhilePhotoFallbackRemainsAnalysable() {
        XCTAssertNil(OfflineTemporalPolicy.exclusion(source: .photo,
                                                      usedFallbackBBox: false))
        XCTAssertNil(OfflineTemporalPolicy.exclusion(source: .photo,
                                                      usedFallbackBBox: true))
        XCTAssertNil(OfflineTemporalPolicy.exclusion(source: .video,
                                                      usedFallbackBBox: false))
        let exclusion = OfflineTemporalPolicy.exclusion(source: .video,
                                                         usedFallbackBBox: true)
        XCTAssertEqual(exclusion, .videoVisionWholeFrameFallback)
        XCTAssertEqual(exclusion?.badgeTitle,
                       "Pose only — excluded from motion analysis")
        XCTAssertEqual(exclusion?.badgeDetail,
                       "Vision found no person box; the full-frame fallback pose is shown "
                       + "for review and was not used for scale, motion, gait, or muscle "
                       + "calculations.")

        let store = OfflineResultStore()
        store.append(OfflineResultStore.FrameResult(
            id: 0, sourceImage: UIImage(), timestamp: 0, status: .success,
            usedFallbackBBox: true, temporalAnalysisExclusion: exclusion,
            camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil,
            isStaticHoldEstimate: false, motionState: .undetermined))
        store.append(OfflineResultStore.FrameResult(
            id: 1, sourceImage: UIImage(), timestamp: 1, status: .success,
            usedFallbackBBox: true,
            camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil,
            isStaticHoldEstimate: false, motionState: .undetermined))

        let muscle = NimbleEngine.MuscleOutput(
            activations: ["soleus_l": 0.4],
            forces: ["soleus_l": 400],
            converged: true,
            timestamp: 0)
        for id in [0, 1] {
            store.replaceBiomechanics(
                forFrameID: id,
                with: OfflineResultStore.BiomechanicsPayload(
                    ikResult: Self.ik(generation: 1),
                    idResult: nil,
                    muscleResult: muscle,
                    isStaticHoldEstimate: true,
                    motionState: .measured(verdict: .hold,
                                           peakSpeedMetersPerSecond: 0,
                                           windowSeconds: 1,
                                           noiseFloorMetersPerSecond: 0.001)))
        }

        XCTAssertFalse(store.frames[0].isEligibleForTemporalAnalysis)
        XCTAssertNil(store.frames[0].ikResult)
        XCTAssertNil(store.frames[0].idResult)
        XCTAssertNil(store.frames[0].muscleResult,
                     "a later route must not bypass video fallback admission")
        XCTAssertFalse(store.frames[0].isStaticHoldEstimate)
        XCTAssertEqual(store.frames[0].motionState, .undetermined)
        XCTAssertFalse(store.frames[0].hasFullBiomechanics)
        XCTAssertTrue(store.frames[1].isEligibleForTemporalAnalysis)
        XCTAssertNotNil(store.frames[1].ikResult)
        XCTAssertNotNil(store.frames[1].muscleResult,
                        "photo fallback retains the existing still-pose path")
        XCTAssertTrue(store.frames[1].hasFullBiomechanics)
    }

    /// A gait pass can revisit a frame after the static pass. Every field on
    /// the stored solve must then come from pass 2: nil means pass 2 withheld
    /// that output, not "keep the pass-1 value".
    @MainActor
    func testBiomechanicsReplacementNeverMixesSolveGenerations() {
        struct Scenario {
            let name: String
            let id: NimbleEngine.IDOutput?
            let muscle: NimbleEngine.MuscleOutput?
        }

        let scenarios = [
            Scenario(name: "pose only", id: nil, muscle: nil),
            Scenario(name: "ID only", id: Self.id(generation: 2), muscle: nil),
            Scenario(name: "full", id: Self.id(generation: 2),
                     muscle: Self.muscle(generation: 2))
        ]

        for (frameID, scenario) in scenarios.enumerated() {
            let store = OfflineResultStore()
            store.append(Self.emptyFrame(id: frameID))
            store.replaceBiomechanics(
                forFrameID: frameID,
                with: OfflineResultStore.BiomechanicsPayload(
                    ikResult: Self.ik(generation: 1),
                    idResult: Self.id(generation: 1),
                    muscleResult: Self.muscle(generation: 1),
                    isStaticHoldEstimate: true,
                    motionState: .measured(verdict: .hold,
                                           peakSpeedMetersPerSecond: 0,
                                           windowSeconds: 1,
                                           noiseFloorMetersPerSecond: 0.001)))

            let passTwoState = OfflineResultStore.MotionState.measured(
                verdict: .movingBeyondStaticBudget,
                peakSpeedMetersPerSecond: 2,
                windowSeconds: 2,
                noiseFloorMetersPerSecond: 0.02)
            store.replaceBiomechanics(
                forFrameID: frameID,
                with: OfflineResultStore.BiomechanicsPayload(
                    ikResult: Self.ik(generation: 2),
                    idResult: scenario.id,
                    muscleResult: scenario.muscle,
                    isStaticHoldEstimate: false,
                    motionState: passTwoState))

            let frame = store.frames[0]
            XCTAssertEqual(frame.ikResult?.jointAngles["generation"], 2,
                           scenario.name)
            XCTAssertEqual(frame.idResult?.jointTorques["generation"],
                           scenario.id?.jointTorques["generation"],
                           scenario.name)
            XCTAssertEqual(frame.muscleResult?.activations["generation"],
                           scenario.muscle?.activations["generation"],
                           scenario.name)
            XCTAssertFalse(frame.isStaticHoldEstimate, scenario.name)
            XCTAssertEqual(frame.motionState, passTwoState, scenario.name)
        }
    }

    /// Ground calibration is a provenance boundary, not a zero-valued solve.
    /// A gait/static second pass must erase the previous generation's physics
    /// and carry the reason in the same atomic replacement.
    @MainActor
    func testGroundUntrustedReplacementErasesPhysicsAndCarriesItsReason() {
        let store = OfflineResultStore()
        store.append(Self.emptyFrame(id: 0))
        store.replaceBiomechanics(
            forFrameID: 0,
            with: OfflineResultStore.BiomechanicsPayload(
                ikResult: Self.ik(generation: 1),
                idResult: Self.id(generation: 1),
                muscleResult: Self.muscle(generation: 1),
                dynamicsAvailability: .available,
                isStaticHoldEstimate: true,
                motionState: .measured(verdict: .hold,
                                       peakSpeedMetersPerSecond: 0,
                                       windowSeconds: 1,
                                       noiseFloorMetersPerSecond: 0.001)))
        XCTAssertTrue(store.frames[0].hasFullBiomechanics)

        store.replaceBiomechanics(
            forFrameID: 0,
            with: OfflineResultStore.BiomechanicsPayload(
                ikResult: Self.ik(generation: 2),
                idResult: nil,
                muscleResult: nil,
                dynamicsAvailability: .groundPlaneUntrusted,
                isStaticHoldEstimate: false,
                motionState: .measured(verdict: .hold,
                                       peakSpeedMetersPerSecond: 0,
                                       windowSeconds: 1,
                                       noiseFloorMetersPerSecond: 0.001)))

        let frame = store.frames[0]
        XCTAssertEqual(frame.ikResult?.jointAngles["generation"], 2)
        XCTAssertNil(frame.idResult)
        XCTAssertNil(frame.muscleResult)
        XCTAssertEqual(frame.dynamicsAvailability, .groundPlaneUntrusted)
        XCTAssertFalse(frame.hasFullBiomechanics)
        XCTAssertEqual(store.groundUntrustedCount, 1)
        XCTAssertTrue(frame.dynamicsAvailability.title.contains("ground"))
        XCTAssertTrue(frame.dynamicsAvailability.detail.contains("30"))
    }

    /// A gait pass replaces the static pass. Clearing first makes a timeout or
    /// missing centred publication fail closed instead of leaving pass-one
    /// torques and muscle output under an analysed-running result.
    @MainActor
    func testGaitReplacementPassClearsOldPhysicsBeforeAnyNewSolve() throws {
        let store = OfflineResultStore()
        store.append(Self.emptyFrame(id: 0))
        store.replaceBiomechanics(
            forFrameID: 0,
            with: OfflineResultStore.BiomechanicsPayload(
                ikResult: Self.ik(generation: 1),
                idResult: Self.id(generation: 1),
                muscleResult: Self.muscle(generation: 1),
                dynamicsAvailability: .available,
                isStaticHoldEstimate: true,
                motionState: .measured(verdict: .hold,
                                       peakSpeedMetersPerSecond: 0,
                                       windowSeconds: 1,
                                       noiseFloorMetersPerSecond: 0.001)))

        store.beginGaitReplacementPass()

        let cleared = store.frames[0]
        XCTAssertEqual(cleared.ikResult?.jointAngles["generation"], 1)
        XCTAssertNil(cleared.idResult)
        XCTAssertNil(cleared.muscleResult)
        XCTAssertEqual(cleared.dynamicsAvailability, .analysisPassIncomplete)
        XCTAssertFalse(cleared.isStaticHoldEstimate)
        XCTAssertFalse(cleared.hasFullBiomechanics)

        let runner = try Self.source(at: "BioMotion/Offline/OfflineSessionRunner.swift")
        let clear = try XCTUnwrap(runner.range(of: "resultStore.beginGaitReplacementPass()"))
        let loop = try XCTUnwrap(runner.range(of: "        for segment in segments {"))
        XCTAssertLessThan(clear.lowerBound, loop.lowerBound,
                          "pass-one physics must be cleared before any pass-two submission")
    }

    /// Filing a solve owns only the biomechanics payload. Image/decoder/model
    /// provenance and frame status remain exactly the envelope that was
    /// appended, even when the new payload contains muscle output.
    @MainActor
    func testBiomechanicsReplacementPreservesFrameEnvelope() {
        let image = UIImage()
        let body = BodyFrame(timestamp: 9, frameNumber: 99, joints: [])
        let store = OfflineResultStore()
        store.append(OfflineResultStore.FrameResult(
            id: 7,
            sourceImage: image,
            timestamp: 8,
            status: .nimbleTimeout,
            usedFallbackBBox: true,
            camT: SIMD3<Float>(1, 2, 3),
            modelChecksums: (input: 10, output: 11, source: 12, bbox: 13, warp: 14),
            bodyFrame: body,
            ikResult: nil,
            idResult: nil,
            muscleResult: nil,
            isStaticHoldEstimate: false,
            motionState: .undetermined))

        store.replaceBiomechanics(
            forFrameID: 7,
            with: OfflineResultStore.BiomechanicsPayload(
                ikResult: Self.ik(generation: 2),
                idResult: Self.id(generation: 2),
                muscleResult: Self.muscle(generation: 2),
                isStaticHoldEstimate: true,
                motionState: .measured(verdict: .hold,
                                       peakSpeedMetersPerSecond: 0,
                                       windowSeconds: 1,
                                       noiseFloorMetersPerSecond: 0.001)))

        let frame = store.frames[0]
        XCTAssertEqual(frame.id, 7)
        XCTAssertTrue(frame.sourceImage === image)
        XCTAssertEqual(frame.timestamp, 8)
        XCTAssertEqual(frame.status, .nimbleTimeout)
        XCTAssertTrue(frame.usedFallbackBBox)
        XCTAssertNil(frame.temporalAnalysisExclusion)
        XCTAssertEqual(frame.camT, SIMD3<Float>(1, 2, 3))
        XCTAssertEqual(frame.modelChecksums?.input, 10)
        XCTAssertEqual(frame.modelChecksums?.output, 11)
        XCTAssertEqual(frame.modelChecksums?.source, 12)
        XCTAssertEqual(frame.modelChecksums?.bbox, 13)
        XCTAssertEqual(frame.modelChecksums?.warp, 14)
        XCTAssertEqual(frame.bodyFrame?.timestamp, 9)
        XCTAssertEqual(frame.bodyFrame?.frameNumber, 99)
        XCTAssertFalse(frame.hasFullBiomechanics,
                       "a non-success envelope must stay fail-closed in UI/load gates")
    }

    // MARK: - The truncation banner

    /// **The case the banner most often fires on, and used to describe
    /// backwards.** A 2 s clip at 30 fps in the 4 s native window: the sampler
    /// wants 120 frames, the clip has 60, so `wasTruncated` is true — and the
    /// old sentence said the clip was too LONG and that 120 frames were used.
    func testAClipShorterThanTheWindowSaysSoAndCountsItsOwnFrames() {
        let duration = 2.0
        let (timestamps, truncated) = FrameSource.sampleTimestamps(
            duration: duration, mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
            nominalFrameRate: 30)
        XCTAssertTrue(truncated, "the sampler still reports it, which is correct")
        XCTAssertEqual(timestamps.count, 60)

        let notice = try? XCTUnwrap(FrameBudgetNotice.make(
            mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
            duration: duration, nominalFrameRate: 30,
            timestamps: timestamps, wasTruncated: truncated))
        XCTAssertEqual(notice?.cause, .clipShorterThanTheWindow)
        XCTAssertEqual(notice?.framesUsed, 60)
        XCTAssertEqual(notice?.analysedSeconds ?? 0, 2.0, accuracy: 0.05)
        let message = notice?.message ?? ""
        print("UI-METRIC short_clip_banner=\(message)")
        XCTAssertTrue(message.contains("shorter"), message)
        XCTAssertFalse(message.contains("longer than the analysis window"), message)
        XCTAssertTrue(message.contains("60"), "the real count, not the budget: \(message)")
        XCTAssertFalse(message.contains("120"), message)
        XCTAssertTrue(message.contains("film for longer") || message.contains("film for longer."),
                      "the lever must be the one that works: \(message)")
    }

    /// At 240 fps the four-second window itself asks for 960 frames, so the
    /// 601-frame run budget binds even when the clip is EXACTLY four seconds.
    /// The notice must name the budget rather than inventing a longer clip.
    /// A genuinely long clip uses the same budget path and remains covered.
    func testTheNativeWindowBudgetNamesItsOwnCauseAtAndBeyondTheWindowBoundary() {
        let mode = FrameSource.SamplingMode.nativeWindow(seconds: FrameSource.analysisWindowSeconds)
        for duration in [30.0, FrameSource.analysisWindowSeconds, 3.0] {
            let (timestamps, truncated) = FrameSource.sampleTimestamps(
                duration: duration, mode: mode, nominalFrameRate: 240)
            XCTAssertTrue(truncated)
            XCTAssertEqual(timestamps.count, FrameSource.maxNativeWindowFrames)
            let notice = try? XCTUnwrap(FrameBudgetNotice.make(
                mode: mode, duration: duration, nominalFrameRate: 240,
                timestamps: timestamps, wasTruncated: truncated))
            XCTAssertEqual(notice?.cause, .budgetCappedTheWindow)
            XCTAssertEqual(notice?.framesUsed, FrameSource.maxNativeWindowFrames)
            XCTAssertEqual(notice?.analysedSeconds ?? 0, 2.5, accuracy: 0.01)
            let message = notice?.message ?? ""
            print("UI-METRIC native_budget_notice duration=\(duration) message=\(message)")
            XCTAssertTrue(message.lowercased().contains("frame budget"), message)
            XCTAssertTrue(message.contains("\(FrameSource.maxNativeWindowFrames)"),
                          "601 frames were used, and 120 is not this mode's budget: \(message)")
            XCTAssertTrue(message.contains("2.5 s"), message)
            XCTAssertFalse(message.contains("\(FrameSource.maxFramesPerRun) frames"), message)
            if duration <= FrameSource.analysisWindowSeconds {
                XCTAssertFalse(message.contains("longer than the analysis window"),
                               "a clip no longer than the window cannot be called longer: \(message)")
            }
        }

        let modeDisclosure = FrameSource.nativeWindowDisclosure
        XCTAssertTrue(modeDisclosure.contains("\(FrameSource.maxNativeWindowFrames)"),
                      "the selector must disclose its high-rate cap: \(modeDisclosure)")
        XCTAssertTrue(modeDisclosure.contains("2.5"),
                      "and the shortest span that cap buys: \(modeDisclosure)")
        XCTAssertFalse(modeDisclosure.lowercased().contains("same number of model calls"),
                       "native-rate cost grows with the selected video's rate: \(modeDisclosure)")
    }

    /// A clip that got everything it asked for says nothing at all.
    func testAnUntruncatedRunShowsNoBanner() {
        let mode = FrameSource.SamplingMode.nativeWindow(seconds: FrameSource.analysisWindowSeconds)
        let (timestamps, truncated) = FrameSource.sampleTimestamps(
            duration: 30.0, mode: mode, nominalFrameRate: 30)
        XCTAssertFalse(truncated)
        XCTAssertEqual(timestamps.count, 120)
        XCTAssertNil(FrameBudgetNotice.make(mode: mode, duration: 30.0, nominalFrameRate: 30,
                                            timestamps: timestamps, wasTruncated: truncated))
        // And a single frame is never a budget story.
        XCTAssertNil(FrameBudgetNotice.make(mode: .singleFrame, duration: 30.0,
                                            nominalFrameRate: 30, timestamps: [15.0],
                                            wasTruncated: true))
    }

    /// The sparse-sampling mode has its own budget and its own arithmetic, and
    /// the same rule decides the cause there.
    /// **The sparse mode's banner, checked on the STRING and not only on the
    /// cause.** This test asserted `notice?.cause` alone while both
    /// native-window tests asserted their message, and the message it was not
    /// asserting was false in both halves: `.fps` has no analysis window (the
    /// cap is `maxFramesPerRun`) and its samples start at `t = 0` and step
    /// forward, so they come from the BEGINNING and not the middle. A user with
    /// a ten-minute clip and a held pose at minute four was told the middle had
    /// been analysed, and trimmed the wrong end.
    func testTheSparseModeIsCappedByItsOwnBudget() throws {
        let mode = FrameSource.SamplingMode.fps(10)
        let (timestamps, truncated) = FrameSource.sampleTimestamps(duration: 600, mode: mode)
        XCTAssertTrue(truncated)
        XCTAssertEqual(timestamps.count, FrameSource.maxFramesPerRun)
        XCTAssertEqual(timestamps.first ?? -1, 0.0, accuracy: 1e-12,
                       "the sparse scan starts at the beginning of the clip")
        let notice = try XCTUnwrap(FrameBudgetNotice.make(
            mode: mode, duration: 600, nominalFrameRate: 30,
            timestamps: timestamps, wasTruncated: truncated))
        XCTAssertEqual(notice.cause, .budgetStoppedTheSparseScan)
        XCTAssertEqual(notice.framesUsed, FrameSource.maxFramesPerRun)
        XCTAssertEqual(notice.analysedSeconds, 12.0, accuracy: 1e-9)

        let message = notice.message
        print("UI-METRIC sparse_budget_notice=\(message)")
        XCTAssertTrue(message.contains("FIRST"), "it names WHERE the frames came from: \(message)")
        XCTAssertFalse(message.lowercased().contains("middle"),
                       "and they did not come from the middle: \(message)")
        XCTAssertFalse(message.lowercased().contains("analysis window"),
                       "there is no analysis window in this mode: \(message)")
        XCTAssertTrue(message.contains("12.0 s"), message)
        XCTAssertTrue(message.contains("600.0 s"), "and the clip's real length: \(message)")

        // Just one sample beyond the cap is still a BUDGET story. The notice
        // used floor(duration / step), called this 120-frame clip-complete, and
        // emitted the native mode's nonexistent four-second-window sentence.
        let boundaryDuration = Double(FrameSource.maxFramesPerRun) / 10.0 + 0.01
        let (boundaryTimestamps, boundaryTruncated) = FrameSource.sampleTimestamps(
            duration: boundaryDuration, mode: mode)
        XCTAssertTrue(boundaryTruncated)
        XCTAssertEqual(boundaryTimestamps.count, FrameSource.maxFramesPerRun)
        let boundaryNotice = try XCTUnwrap(FrameBudgetNotice.make(
            mode: mode, duration: boundaryDuration, nominalFrameRate: 30,
            timestamps: boundaryTimestamps, wasTruncated: boundaryTruncated))
        XCTAssertEqual(boundaryNotice.cause, .budgetStoppedTheSparseScan)
        XCTAssertTrue(boundaryNotice.message.contains("FIRST"), boundaryNotice.message)
        XCTAssertFalse(boundaryNotice.message.lowercased().contains("analysis window"),
                       boundaryNotice.message)
    }

    // MARK: - Fixtures

    private static func emptyFrame(id: Int) -> OfflineResultStore.FrameResult {
        OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id), status: .success,
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil,
            isStaticHoldEstimate: false, motionState: .undetermined)
    }

    private static func source(at relativePath: String) throws -> String {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        return try String(
            contentsOf: repositoryRoot.appendingPathComponent(relativePath),
            encoding: .utf8)
    }

    private static func ik(generation: Double) -> NimbleEngine.IKOutput {
        NimbleEngine.IKOutput(
            jointAngles: ["generation": generation],
            markerRMSMeters: generation,
            ikLossSquaredMeters: generation,
            timestamp: generation)
    }

    private static func id(generation: Double) -> NimbleEngine.IDOutput {
        NimbleEngine.IDOutput(jointTorques: ["generation": generation],
                              timestamp: generation)
    }

    private static func muscle(generation: Double) -> NimbleEngine.MuscleOutput {
        NimbleEngine.MuscleOutput(
            activations: ["generation": generation],
            forces: ["generation": generation],
            converged: true,
            timestamp: generation)
    }

    static func stanceFrame(id: Int = 0, side: Int,
                            solverLeft: Bool? = nil, solverRight: Bool? = nil,
                            cleanWindow: Bool = true) -> OfflineResultStore.FrameResult {
        gaitFrame(id: id, contactSide: side,
                  solverLeft: solverLeft ?? (side < 0),
                  solverRight: solverRight ?? (side > 0),
                  cleanWindow: cleanWindow)
    }

    static func gaitFrame(id: Int = 0, contactSide: Int,
                          solverLeft: Bool, solverRight: Bool,
                          cleanWindow: Bool,
                          dynamicsAvailability: NimbleEngine.DynamicsAvailability? = nil)
        -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.1,
            residualInBodyWeights: 0.1,
            contactSide: contactSide,
            contactIndex: contactSide < 0 ? 0 : 1,
            solverSawLeftContact: solverLeft,
            solverSawRightContact: solverRight,
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: cleanWindow)
        let muscle = NimbleEngine.MuscleOutput(
            activations: ["soleus_l": 0.5, "soleus_r": 0.5],
            forces: ["soleus_l": 500, "soleus_r": 500],
            converged: true, timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            dynamicsAvailability: dynamicsAvailability,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: contactSide == 0 ? .gaitFlight : .gaitStance,
                               outcome: outcome))
    }
}
