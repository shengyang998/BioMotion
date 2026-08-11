import UIKit
import Combine
import XCTest

@testable import BioMotion

final class CameraReferenceProjectionTests: XCTestCase {

    @MainActor
    func testEngineResetEmitsOneCoherentResultSnapshotNotification() {
        let engine = NimbleEngine()
        var snapshots: [(hasIK: Bool, hasID: Bool, hasMuscle: Bool,
                         availability: NimbleEngine.DynamicsAvailability)] = []
        let cancellable = engine.objectWillChange.sink {
            snapshots.append((
                engine.lastIKResult != nil,
                engine.lastIDResult != nil,
                engine.lastMuscleResult != nil,
                engine.dynamicsAvailability
            ))
        }

        let lease = engine.acquireOfflinePolicyLease()
        XCTAssertEqual(snapshots.count, 1,
                       "reset state must not publish one callback per field")
        XCTAssertFalse(snapshots[0].hasIK)
        XCTAssertFalse(snapshots[0].hasID)
        XCTAssertFalse(snapshots[0].hasMuscle)
        XCTAssertEqual(snapshots[0].availability, .waitingForMotionWindow)

        snapshots.removeAll()
        XCTAssertTrue(engine.resetSessionState(offlinePolicyLease: lease))
        XCTAssertEqual(snapshots.count, 1)
        XCTAssertEqual(snapshots[0].availability, .waitingForMotionWindow)
        withExtendedLifetime(cancellable) {}
    }

    @MainActor
    func testUnknownCameraFailsClosedWithoutErasingPose() {
        let store = OfflineResultStore()
        store.setValidatedFootContactSupport(true)
        store.append(Self.availableFrame(isStatic: true))

        let frame = store.frames[0]
        XCTAssertEqual(store.cameraReferenceState, .unmeasured)
        XCTAssertEqual(frame.dynamicsAvailability, .cameraReferenceUnavailable)
        XCTAssertNotNil(frame.ikResult)
        XCTAssertNil(frame.idResult)
        XCTAssertNil(frame.muscleResult)
        XCTAssertFalse(frame.isStaticHoldEstimate)
    }

    @MainActor
    func testSingleFramePermissionAllowsOnlyStaticEquilibrium() {
        let staticStore = OfflineResultStore()
        staticStore.setCameraReferenceState(.notRequiredForSingleFrame)
        staticStore.setValidatedFootContactSupport(true)
        staticStore.append(Self.availableFrame(isStatic: true))
        XCTAssertTrue(staticStore.frames[0].hasFullBiomechanics)

        let temporalStore = OfflineResultStore()
        temporalStore.setCameraReferenceState(.notRequiredForSingleFrame)
        temporalStore.setValidatedFootContactSupport(true)
        temporalStore.append(Self.availableFrame(isStatic: false))
        XCTAssertEqual(temporalStore.frames[0].dynamicsAvailability,
                       .cameraReferenceUnavailable)
        XCTAssertNil(temporalStore.frames[0].idResult)
        XCTAssertNil(temporalStore.frames[0].muscleResult)
    }

    @MainActor
    func testSingleFrameStaticPermissionRejectsAContradictoryNonHoldFrame() {
        let store = OfflineResultStore()
        store.setCameraReferenceState(.notRequiredForSingleFrame)
        store.setValidatedFootContactSupport(true)
        store.append(Self.availableFrame(
            isStatic: true,
            motionState: .gait(verdict: .gaitStance, outcome: nil)
        ))

        let frame = store.frames[0]
        XCTAssertEqual(frame.dynamicsAvailability, .cameraReferenceUnavailable)
        XCTAssertNil(frame.idResult)
        XCTAssertNil(frame.muscleResult)
        XCTAssertFalse(frame.isStaticHoldEstimate)
    }

    @MainActor
    func testOnlyCalibratedStaticVideoStateAllowsTemporalDynamics() {
        let states: [(CameraReferenceState, Bool)] = [
            (.unmeasured, false),
            (.moving(Self.evidence), false),
            (.betweenCalibrationBands(Self.evidence), false),
            (.calibrationRequired(Self.uncalibratedEvidence), false),
            (.calibrationUnavailable, false),
            (.indeterminate(.insufficientBackground), false),
            (.staticWithinBudget(Self.evidence), true),
        ]
        for (index, entry) in states.enumerated() {
            let store = OfflineResultStore()
            store.setCameraReferenceState(entry.0)
            store.setValidatedFootContactSupport(true)
            store.append(Self.availableFrame(id: index, isStatic: false))
            XCTAssertEqual(store.frames[0].hasFullBiomechanics, entry.1,
                           "camera state \(entry.0)")
            XCTAssertEqual(store.frames[0].dynamicsAvailability,
                           entry.1 ? .available : .cameraReferenceUnavailable)
        }
    }

    @MainActor
    func testLateCameraDowngradeAtomicallyPurgesLoadsButKeepsKinematics() {
        let store = OfflineResultStore()
        store.setCameraReferenceState(.staticWithinBudget(Self.evidence))
        store.setValidatedFootContactSupport(true)
        store.append(Self.availableFrame(isStatic: true))
        XCTAssertTrue(store.frames[0].hasFullBiomechanics)

        store.setCameraReferenceState(.moving(Self.evidence))

        let frame = store.frames[0]
        XCTAssertEqual(store.cameraReferenceState, .moving(Self.evidence))
        XCTAssertEqual(frame.dynamicsAvailability, .cameraReferenceUnavailable)
        XCTAssertNotNil(frame.bodyFrame)
        XCTAssertNotNil(frame.ikResult)
        XCTAssertNil(frame.idResult)
        XCTAssertNil(frame.muscleResult)
        XCTAssertFalse(frame.isStaticHoldEstimate)
    }

    @MainActor
    func testCameraDowngradePublishesOnlyCoherentSnapshotsAndReentrantLatestWins() {
        let store = OfflineResultStore()
        store.setCameraReferenceState(.staticWithinBudget(Self.evidence))
        store.setValidatedFootContactSupport(true)
        store.append(Self.availableFrame(isStatic: true))
        XCTAssertNotNil(store.frames[0].idResult)

        var snapshots: [(state: CameraReferenceState, hasID: Bool)] = []
        var didReenter = false
        let cancellable = store.objectWillChange.sink {
            snapshots.append((
                state: store.cameraReferenceState,
                hasID: store.frames.first?.idResult != nil
            ))
            if !didReenter {
                didReenter = true
                store.setCameraReferenceState(.calibrationUnavailable)
            }
        }

        store.setCameraReferenceState(.moving(Self.evidence))

        XCTAssertEqual(store.cameraReferenceState, .calibrationUnavailable,
                       "the reentrant, later mutation must win")
        XCTAssertEqual(snapshots.count, 2,
                       "each complete store transaction must emit once")
        XCTAssertEqual(snapshots.map(\.state), [
            .moving(Self.evidence),
            .calibrationUnavailable,
        ])
        XCTAssertTrue(snapshots.allSatisfy { !$0.hasID },
                      "no observer may see denied camera state beside stale dynamics")
        withExtendedLifetime(cancellable) {}
    }

    @MainActor
    func testStaticIDWithoutMuscleRetainsItsSolveClassAcrossReprojection() {
        let store = OfflineResultStore()
        store.setCameraReferenceState(.notRequiredForSingleFrame)
        store.setValidatedFootContactSupport(true)
        store.append(Self.availableFrame(isStatic: true, includesMuscle: false))

        XCTAssertNotNil(store.frames[0].idResult)
        XCTAssertNil(store.frames[0].muscleResult)
        XCTAssertTrue(store.frames[0].isStaticHoldEstimate,
                      "static-equilibrium provenance belongs to ID, not muscle availability")

        store.setCameraReferenceState(.notRequiredForSingleFrame)
        XCTAssertNotNil(store.frames[0].idResult,
                        "reprojection must not reinterpret static ID as temporal dynamics")
        XCTAssertTrue(store.frames[0].isStaticHoldEstimate)
    }

    @MainActor
    func testFrameSelectionPublishesOnceAndIgnoresStaleIndices() {
        let store = OfflineResultStore()
        store.append(Self.availableFrame(id: 0, isStatic: false))
        store.append(Self.availableFrame(id: 1, isStatic: false))
        XCTAssertEqual(store.selectedIndex, 1)

        var publishedSelections: [Int] = []
        let cancellable = store.objectWillChange.sink {
            publishedSelections.append(store.selectedIndex)
        }
        store.selectFrame(at: 0)
        store.selectFrame(at: 0)
        store.selectFrame(at: -1)
        store.selectFrame(at: 99)

        XCTAssertEqual(store.selectedIndex, 0)
        XCTAssertEqual(publishedSelections, [0])
        withExtendedLifetime(cancellable) {}
    }

    @MainActor
    func testPermanentContactCapabilityStillHasPriorityAndExistingReasonsSurvive() {
        let unsupported = OfflineResultStore()
        unsupported.setCameraReferenceState(.moving(Self.evidence))
        unsupported.setValidatedFootContactSupport(false)
        unsupported.append(Self.availableFrame(isStatic: true))
        XCTAssertEqual(unsupported.frames[0].dynamicsAvailability,
                       .contactSupportUnavailable,
                       "a tripod must not appear able to unlock a missing model capability")

        let alreadyWithheld = OfflineResultStore()
        alreadyWithheld.setCameraReferenceState(.moving(Self.evidence))
        alreadyWithheld.setValidatedFootContactSupport(true)
        var frame = Self.availableFrame(isStatic: true)
        frame = OfflineResultStore.FrameResult(
            id: frame.id, sourceImage: frame.sourceImage, timestamp: frame.timestamp,
            status: frame.status, usedFallbackBBox: frame.usedFallbackBBox,
            camT: frame.camT, modelChecksums: frame.modelChecksums,
            bodyFrame: frame.bodyFrame, ikResult: frame.ikResult,
            idResult: nil, muscleResult: nil,
            dynamicsAvailability: .withheld(.movingBeyondStaticBudget),
            isStaticHoldEstimate: false,
            motionState: .measured(verdict: .movingBeyondStaticBudget,
                                   peakSpeedMetersPerSecond: 0.3,
                                   windowSeconds: 0.25,
                                   noiseFloorMetersPerSecond: 0.01)
        )
        alreadyWithheld.append(frame)
        XCTAssertEqual(alreadyWithheld.frames[0].dynamicsAvailability,
                       .withheld(.movingBeyondStaticBudget))
    }

    @MainActor
    func testResetReturnsCameraStateToFailClosedDefault() {
        let store = OfflineResultStore()
        store.setCameraReferenceState(.staticWithinBudget(Self.evidence))
        store.reset()
        XCTAssertEqual(store.cameraReferenceState, .unmeasured)
    }

    func testCameraBannerNamesMeasuredBoundaryWithoutClaimingPhysicalStillness() {
        let staticTitle = CameraReferenceState.staticWithinBudget(Self.evidence).bannerTitle
        let staticDetail = CameraReferenceState.staticWithinBudget(Self.evidence).bannerDetail
        XCTAssertTrue(staticTitle?.contains("Visible background") == true)
        XCTAssertFalse(staticTitle?.lowercased().contains("camera is stationary") == true)
        XCTAssertTrue(staticDetail?.contains("1.0%") == true, staticDetail ?? "")
        XCTAssertTrue(staticDetail?.contains("rotation") == true, staticDetail ?? "")

        let calibration = CameraReferenceState
            .calibrationRequired(Self.uncalibratedEvidence)
        XCTAssertTrue(calibration.bannerTitle?.lowercased().contains("calibration") == true)
        XCTAssertTrue(calibration.bannerDetail?.contains("Pose, anatomy and contact timing") == true)
        XCTAssertFalse(calibration.bannerDetail?.contains("stable") == true)
    }

    func testCameraAnalysisWindowMatchesTheSelectedDerivativeCadence() {
        XCTAssertNil(CameraAnalysisPolicy.derivativeWindowSeconds(
            samplingMode: .singleFrame, nominalFrameRate: 30))
        XCTAssertEqual(CameraAnalysisPolicy.derivativeWindowSeconds(
            samplingMode: .fps(10), nominalFrameRate: 30) ?? -1,
                       0.8, accuracy: 1e-12)
        XCTAssertEqual(CameraAnalysisPolicy.derivativeWindowSeconds(
            samplingMode: .nativeWindow(seconds: 4), nominalFrameRate: 30) ?? -1,
                       8.0 / 30.0, accuracy: 1e-12)

        let range = CameraAnalysisPolicy.analysisRange(
            requestedTimestamps: [2.0, 2.1, 2.2],
            assetDuration: 3,
            nominalFrameRate: 30
        )
        XCTAssertEqual(range?.startSeconds ?? -1, 2, accuracy: 1e-12)
        XCTAssertEqual(range?.endSeconds ?? -1, 2.2 + 1.0 / 30.0,
                       accuracy: 1e-12)
    }

    func testRunnerFinalizesCameraBeforePublishingFramesAndGatesGaitReplacement() throws {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let source = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Offline/OfflineSessionRunner.swift"),
            encoding: .utf8
        )
        let setCamera = try XCTUnwrap(source.range(
            of: "resultStore.setCameraReferenceState(cameraState)"))
        let frameLoop = try XCTUnwrap(source.range(of: "        for (i, frame) in decoded.enumerated()"))
        XCTAssertLessThan(setCamera.lowerBound, frameLoop.lowerBound)

        let gaitStart = try XCTUnwrap(source.range(
            of: "    private func runGaitPassIfThisIsARun("))
        let gaitBody = source[gaitStart.lowerBound...]
        let cameraSnapshot = try XCTUnwrap(gaitBody.range(
            of: "let cameraPermitsTemporalDynamics ="))
        let cameraCondition = try XCTUnwrap(gaitBody.range(
            of: "if supportsFootContact && cameraPermitsTemporalDynamics"))
        let replacement = try XCTUnwrap(gaitBody.range(
            of: "resultStore.beginGaitReplacementPass()"))
        let publishTiming = try XCTUnwrap(gaitBody.range(
            of: "resultStore.setGait(.analysed(report: timingReport))"))
        let cameraGate = try XCTUnwrap(gaitBody.range(
            of: "guard cameraPermitsTemporalDynamics else { return }"))
        XCTAssertLessThan(cameraSnapshot.lowerBound, replacement.lowerBound)
        XCTAssertLessThan(cameraCondition.lowerBound, replacement.lowerBound)
        XCTAssertLessThan(publishTiming.lowerBound, cameraGate.lowerBound,
                          "kinematic gait timing publishes before camera blocks dynamics")
    }

    func testRunOwnershipHasIdempotentInvalidateAndCompleteSemantics() {
        var ownership = OfflineRunOwnership()
        XCTAssertNil(ownership.activeToken)
        XCTAssertNil(ownership.invalidateCurrent())

        let first = ownership.beginRun()
        XCTAssertEqual(ownership.activeToken, first)
        XCTAssertTrue(ownership.isCurrent(first))
        XCTAssertEqual(ownership.invalidateCurrent(), first)
        XCTAssertNil(ownership.activeToken)
        XCTAssertNil(ownership.invalidateCurrent(),
                     "repeated cancellation must not manufacture a new fence")
        XCTAssertFalse(ownership.complete(first),
                       "an invalidated task's defer no longer owns cleanup")

        let second = ownership.beginRun()
        XCTAssertNotEqual(first, second)
        XCTAssertFalse(ownership.isCurrent(first))
        XCTAssertTrue(ownership.isCurrent(second))
        XCTAssertTrue(ownership.complete(second))
        XCTAssertNil(ownership.activeToken)
        XCTAssertFalse(ownership.complete(second),
                       "completion must be safe when defer/backstops repeat")
    }

    func testFrameReceiptCarriesExactGenerationAndMonotonicSubmissionIdentity() {
        let first = NimbleEngine.FrameReceipt(generation: 4, submissionID: 18)
        let second = NimbleEngine.FrameReceipt(generation: 4, submissionID: 19)

        XCTAssertNotEqual(first, second)
        XCTAssertEqual(first.generation, 4)
        XCTAssertEqual(first.submissionID, 18)
        XCTAssertEqual(
            NimbleEngine.FrameSubmission.accepted(first),
            .accepted(first)
        )
        XCTAssertEqual(
            NimbleEngine.FrameCompletion(receipt: second, status: .superseded),
            .init(receipt: second, status: .superseded)
        )
    }

    func testCameraStateMapsToSeparateStaticAndTemporalEngineAuthorization() {
        let live = NimbleEngine.CameraDynamicsAuthorization.unrestricted
        XCTAssertTrue(live.permits(.staticEquilibrium))
        XCTAssertTrue(live.permits(.temporal))

        let single = OfflineSessionRunner.cameraDynamicsAuthorization(
            for: .notRequiredForSingleFrame
        )
        XCTAssertTrue(single.permits(.staticEquilibrium))
        XCTAssertFalse(single.permits(.temporal))

        let calibratedStatic = OfflineSessionRunner.cameraDynamicsAuthorization(
            for: .staticWithinBudget(Self.evidence)
        )
        XCTAssertTrue(calibratedStatic.permits(.staticEquilibrium))
        XCTAssertTrue(calibratedStatic.permits(.temporal))

        let deniedStates: [CameraReferenceState] = [
            .unmeasured,
            .moving(Self.evidence),
            .betweenCalibrationBands(Self.evidence),
            .calibrationRequired(Self.uncalibratedEvidence),
            .calibrationUnavailable,
            .indeterminate(.registrationFailed),
        ]
        for state in deniedStates {
            let authorization = OfflineSessionRunner.cameraDynamicsAuthorization(for: state)
            XCTAssertFalse(authorization.permits(.staticEquilibrium), "\(state)")
            XCTAssertFalse(authorization.permits(.temporal), "\(state)")
        }
    }

    func testContactCapabilityPrecedesCameraAuthorizationAtEngineBoundary() {
        let denied = NimbleEngine.CameraDynamicsAuthorization.denied
        XCTAssertEqual(
            NimbleEngine.dynamicsPreflightAvailability(
                hasValidatedFootContactSupport: false,
                cameraAuthorization: denied,
                dynamicsReference: .mhrRootRelative,
                solveClass: .staticEquilibrium
            ),
            .contactSupportUnavailable
        )
        XCTAssertEqual(
            NimbleEngine.dynamicsPreflightAvailability(
                hasValidatedFootContactSupport: true,
                cameraAuthorization: denied,
                dynamicsReference: .mhrRootRelative,
                solveClass: .staticEquilibrium
            ),
            .cameraReferenceUnavailable
        )
        XCTAssertEqual(
            NimbleEngine.dynamicsPreflightAvailability(
                hasValidatedFootContactSupport: true,
                cameraAuthorization: .unrestricted,
                dynamicsReference: .mhrCameraRelativePosition,
                solveClass: .staticEquilibrium
            ),
            .gravityReferenceUnavailable
        )
        let gravityOnly = BodyFrame.DynamicsReference(
            gravity: .gravityAligned,
            rootTrajectory: .cameraRelativePositionOnly
        )
        XCTAssertNil(NimbleEngine.dynamicsPreflightAvailability(
            hasValidatedFootContactSupport: true,
            cameraAuthorization: .unrestricted,
            dynamicsReference: gravityOnly,
            solveClass: .staticEquilibrium
        ), "a gravity-aligned pose is sufficient for explicitly static equilibrium")
        XCTAssertEqual(NimbleEngine.dynamicsPreflightAvailability(
            hasValidatedFootContactSupport: true,
            cameraAuthorization: .unrestricted,
            dynamicsReference: gravityOnly,
            solveClass: .temporal
        ), .rootTrajectoryUnavailable)
        XCTAssertEqual(NimbleEngine.dynamicsPreflightAvailability(
            hasValidatedFootContactSupport: true,
            cameraAuthorization: .unrestricted,
            dynamicsReference: .liveARKit,
            solveClass: .temporal
        ), .rootTrajectoryUnavailable,
        "an ARKit position stream needs measured continuity/noise evidence")
        XCTAssertNil(NimbleEngine.dynamicsPreflightAvailability(
            hasValidatedFootContactSupport: true,
            cameraAuthorization: .unrestricted,
            dynamicsReference: .dynamicsQualifiedWorld,
            solveClass: .temporal
        ))

        XCTAssertFalse(NimbleEngine.dynamicsReferenceTransitionRequiresReset(
            from: nil, to: .liveARKit),
            "the first reference starts an empty history")
        XCTAssertFalse(NimbleEngine.dynamicsReferenceTransitionRequiresReset(
            from: .liveARKit, to: .liveARKit))
        XCTAssertTrue(NimbleEngine.dynamicsReferenceTransitionRequiresReset(
            from: .liveARKit, to: .dynamicsQualifiedWorld),
            "an authorization upgrade cannot reuse unqualified SG samples")
        XCTAssertTrue(NimbleEngine.dynamicsReferenceTransitionRequiresReset(
            from: .dynamicsQualifiedWorld, to: .mhrRootRelative),
            "a coordinate-space change cannot reuse IK/ground history")
        XCTAssertTrue(NimbleEngine.DynamicsAvailability
            .referenceTransitionWarmup.invalidatesPreviousDynamics,
            "a rebuilt derivative window must erase dynamics from the old reference")
        XCTAssertTrue(NimbleEngine.DynamicsAvailability
            .referenceTransitionWarmup.detail.contains("reference changed"))
    }

    func testRunnerAcquiresLeaseBeforeLaunchingTaskAndRechecksSuspensions() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let runStart = try XCTUnwrap(source.range(
            of: "    func run(source: RunSource, samplingMode: FrameSource.SamplingMode)"))
        let runInternalStart = try XCTUnwrap(source.range(
            of: "    private func runInternal(",
            range: runStart.upperBound..<source.endIndex))
        let runBody = source[runStart.lowerBound..<runInternalStart.lowerBound]
        let fence = try XCTUnwrap(runBody.range(of: "fenceCurrentRun("))
        let lease = try XCTUnwrap(runBody.range(
            of: "let token = runOwnership.beginRun()"))
        let engineLease = try XCTUnwrap(runBody.range(
            of: "let engineLease = nimble.acquireOfflinePolicyLease()"))
        let launch = try XCTUnwrap(runBody.range(of: "runTask = Task"))
        XCTAssertLessThan(fence.lowerBound, lease.lowerBound)
        XCTAssertLessThan(lease.lowerBound, engineLease.lowerBound)
        XCTAssertLessThan(engineLease.lowerBound, launch.lowerBound)
        XCTAssertTrue(runBody.contains("Task { [self] in"),
                      "the task must retain Runner until conditional defer is installed")

        let runInternalEnd = try XCTUnwrap(source.range(
            of: "    // MARK: - Gait pass",
            range: runInternalStart.upperBound..<source.endIndex))
        let runInternal = source[runInternalStart.lowerBound..<runInternalEnd.lowerBound]
        XCTAssertFalse(runInternal.contains("beginRun()"),
                       "an actor-reentrant task must not acquire ownership after launch")
        XCTAssertTrue(runInternal.contains("guard isRunActive(token) else"))
        let cleanup = try XCTUnwrap(runInternal.range(of: "        defer {"))
        let initialGuard = try XCTUnwrap(runInternal.range(
            of: "        guard isRunActive(token) else"))
        XCTAssertLessThan(cleanup.lowerBound, initialGuard.lowerBound,
                          "lease cleanup must exist even if a second Runner wins before Task start")
        XCTAssertTrue(source.contains("try await Task.sleep(nanoseconds: 150_000_000)"))
        XCTAssertFalse(source.contains("try? await Task.sleep(nanoseconds: 150_000_000)"))
    }

    func testRunnerResolvesContactAndCalibrationBeforeOpeningCameraAdapter() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let runStart = try XCTUnwrap(source.range(
            of: "    func run(source: RunSource, samplingMode: FrameSource.SamplingMode)"))
        let runInternalStart = try XCTUnwrap(source.range(
            of: "    private func runInternal(",
            range: runStart.upperBound..<source.endIndex))
        let runBody = source[runStart.lowerBound..<runInternalStart.lowerBound]
        XCTAssertTrue(runBody.contains("phase = .loadingModel"))
        XCTAssertFalse(runBody.contains("phase = .checkingCameraReference"),
                       "camera work must not be announced before contact admission")

        let gaitStart = try XCTUnwrap(source.range(
            of: "    // MARK: - Gait pass",
            range: runInternalStart.upperBound..<source.endIndex))
        let runInternal = source[runInternalStart.lowerBound..<gaitStart.lowerBound]
        let capability = try XCTUnwrap(runInternal.range(
            of: "let hasValidatedFootContactSupport ="))
        let admission = try XCTUnwrap(runInternal.range(
            of: "CameraReferenceAnalysisAdmission.decide("))
        let resolvedWithoutAdapter = try XCTUnwrap(runInternal.range(
            of: "if let resolvedState = cameraAdmission.resolvedState"))
        let adapter = try XCTUnwrap(runInternal.range(
            of: "cameraState = try await resolveCameraReference("))
        let finalized = try XCTUnwrap(runInternal.range(
            of: "resultStore.setCameraReferenceState(cameraState)"))
        XCTAssertLessThan(capability.lowerBound, admission.lowerBound)
        XCTAssertLessThan(admission.lowerBound, resolvedWithoutAdapter.lowerBound)
        XCTAssertLessThan(resolvedWithoutAdapter.lowerBound, adapter.lowerBound)
        XCTAssertLessThan(adapter.lowerBound, finalized.lowerBound)
        XCTAssertTrue(runInternal.contains(
            "cameraMotionAnalyzer.isVersionedCalibrationReady"))
    }

    func testRunnerRechecksOwnershipAfterSynchronousStorePublications() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")

        let cameraStore = try XCTUnwrap(source.range(
            of: "resultStore.setCameraReferenceState(cameraState)"))
        let cameraWrite = try XCTUnwrap(source.range(
            of: "nimble.cameraDynamicsAuthorization =",
            range: cameraStore.upperBound..<source.endIndex))
        let cameraSeam = source[cameraStore.upperBound..<cameraWrite.lowerBound]
        XCTAssertTrue(cameraSeam.contains("guard isRunActive(token) else"),
                      "a synchronous store subscriber may replace this run")

        let gaitStore = try XCTUnwrap(source.range(
            of: "resultStore.setGait(.analysed(report: timingReport))"))
        let passReset = try XCTUnwrap(source.range(
            of: "nimble.resetAnalysisPassStatePreservingGround(",
            range: gaitStore.upperBound..<source.endIndex))
        let gaitSeam = source[gaitStore.upperBound..<passReset.lowerBound]
        let ownership = try XCTUnwrap(gaitSeam.range(
            of: "guard isRunActive(token),"))
        let leaseRead = try XCTUnwrap(gaitSeam.range(
            of: "let enginePolicyLease"))
        XCTAssertLessThan(ownership.lowerBound, leaseRead.lowerBound,
                          "a stale task must not borrow its successor's mutable lease property")
    }

    func testRunnerRechecksExactLeaseAfterPassResetBeforePolicyMutation() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let reset = try XCTUnwrap(source.range(
            of: "nimble.resetAnalysisPassStatePreservingGround("))
        let mutation = try XCTUnwrap(source.range(
            of: "nimble.staticHoldGating = false",
            range: reset.upperBound..<source.endIndex))
        let seam = source[reset.upperBound..<mutation.lowerBound]

        XCTAssertTrue(seam.contains("guard isRunActive(token),"),
                      "reset notifications may synchronously replace the run")
        XCTAssertTrue(seam.contains("self.enginePolicyLease == enginePolicyLease"),
                      "the old task must retain the exact captured lease")
        XCTAssertTrue(seam.contains("nimble.ownsOfflinePolicyLease(enginePolicyLease)"),
                      "engine ownership may change while reset notifications unwind")
    }

    func testRunnerRetiresOrRechecksOwnershipAcrossEngineNotifications() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        XCTAssertFalse(source.contains("@Published private(set) var phase"))
        XCTAssertFalse(source.contains("@Published private(set) var frameBudgetNotice"))
        XCTAssertTrue(source.contains("objectWillChange.send()"),
                      "runner state must notify only after its stored transaction commits")

        let runStart = try XCTUnwrap(source.range(
            of: "    func run(source:"))
        let fenceStart = try XCTUnwrap(source.range(
            of: "    private func fenceCurrentRun(",
            range: runStart.upperBound..<source.endIndex))
        let run = source[runStart.lowerBound..<fenceStart.lowerBound]
        let acquire = try XCTUnwrap(run.range(
            of: "let engineLease = nimble.acquireOfflinePolicyLease()"))
        let localLeaseWrite = try XCTUnwrap(run.range(
            of: "enginePolicyLease = engineLease",
            range: acquire.upperBound..<run.endIndex))
        let acquireSeam = run[acquire.upperBound..<localLeaseWrite.lowerBound]
        XCTAssertTrue(acquireSeam.contains("runOwnership.isCurrent(token)"))
        XCTAssertTrue(acquireSeam.contains("nimble.ownsOfflinePolicyLease(engineLease)"))

        let fenceEnd = try XCTUnwrap(source.range(
            of: "    private func isRunActive(",
            range: fenceStart.upperBound..<source.endIndex))
        let fence = source[fenceStart.lowerBound..<fenceEnd.lowerBound]
        XCTAssertTrue(fence.contains("if enginePolicyLease == lease"),
                      "release notification must not clear a successor lease")
        XCTAssertTrue(fence.contains("runOwnership.activeToken == nil"),
                      "cancel publication must not overwrite a reentrant successor")

        let internalStart = try XCTUnwrap(source.range(
            of: "    private func runInternal("))
        let internalBody = source[internalStart.lowerBound...]
        let release = try XCTUnwrap(internalBody.range(
            of: "nimble.releaseOfflinePolicyLease(engineLease)"))
        let deferPrefix = internalBody[..<release.lowerBound]
        XCTAssertTrue(deferPrefix.contains("runOwnership.complete(token)"))
        XCTAssertTrue(deferPrefix.contains("enginePolicyLease = nil"))
        XCTAssertTrue(deferPrefix.contains("runTask = nil"),
                      "local handles must retire before release can synchronously reenter")
    }

    func testLatestRunInvocationWinsAcrossFenceReentrancyAndFailedLaunchRetires() throws {
        var ownership = OfflineRunOwnership()
        let outer = ownership.beginInvocation()
        let reentrant = ownership.beginInvocation()
        XCTAssertFalse(ownership.isLatestInvocation(outer))
        XCTAssertTrue(ownership.isLatestInvocation(reentrant))

        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let runStart = try XCTUnwrap(source.range(of: "    func run(source:"))
        let fenceStart = try XCTUnwrap(source.range(
            of: "    private func fenceCurrentRun(",
            range: runStart.upperBound..<source.endIndex))
        let run = source[runStart.lowerBound..<fenceStart.lowerBound]
        let invocation = try XCTUnwrap(run.range(of: "runOwnership.beginInvocation()"))
        let fence = try XCTUnwrap(run.range(of: "fenceCurrentRun(markCancelled: false)"))
        let latest = try XCTUnwrap(run.range(
            of: "runOwnership.isLatestInvocation(invocation)",
            range: fence.upperBound..<run.endIndex))
        let token = try XCTUnwrap(run.range(
            of: "runOwnership.beginRun()",
            range: latest.upperBound..<run.endIndex))
        XCTAssertLessThan(invocation.lowerBound, fence.lowerBound)
        XCTAssertLessThan(fence.lowerBound, latest.lowerBound)
        XCTAssertLessThan(latest.lowerBound, token.lowerBound)

        XCTAssertGreaterThanOrEqual(
            run.components(separatedBy: "retireUnlaunchedRun(").count - 1,
            2,
            "both post-acquire guards need exact conditional cleanup"
        )

        let cancelStart = try XCTUnwrap(source.range(of: "    func cancel()"))
        let runEntry = try XCTUnwrap(source.range(
            of: "    func run(source:",
            range: cancelStart.upperBound..<source.endIndex))
        let cancel = source[cancelStart.lowerBound..<runEntry.lowerBound]
        let cancelInvocation = try XCTUnwrap(cancel.range(
            of: "runOwnership.beginInvocation()"))
        let cancelFence = try XCTUnwrap(cancel.range(
            of: "fenceCurrentRun(markCancelled: true)"))
        XCTAssertLessThan(cancelInvocation.lowerBound, cancelFence.lowerBound,
                          "Cancel must invalidate an outer run before publishing its fence")

        let retireStart = try XCTUnwrap(source.range(
            of: "    private func retireUnlaunchedRun("))
        let retireEnd = try XCTUnwrap(source.range(
            of: "    /// Synchronous lifecycle fence",
            range: retireStart.upperBound..<source.endIndex))
        let retire = source[retireStart.lowerBound..<retireEnd.lowerBound]
        let complete = try XCTUnwrap(retire.range(of: "runOwnership.complete(token)"))
        let localLeaseClear = try XCTUnwrap(retire.range(
            of: "enginePolicyLease = nil",
            range: complete.upperBound..<retire.endIndex))
        let notify = try XCTUnwrap(retire.range(
            of: "objectWillChange.send()",
            range: localLeaseClear.upperBound..<retire.endIndex))
        let release = try XCTUnwrap(retire.range(
            of: "nimble.releaseOfflinePolicyLease(engineLease)",
            range: notify.upperBound..<retire.endIndex))
        XCTAssertLessThan(complete.lowerBound, localLeaseClear.lowerBound)
        XCTAssertLessThan(localLeaseClear.lowerBound, notify.lowerBound)
        XCTAssertLessThan(notify.lowerBound, release.lowerBound)
    }

    func testSegmentResetAndModelScaleKeepTheCapturedRunLease() throws {
        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let segmentStart = try XCTUnwrap(runner.range(
            of: "    private func endTemporalSegment("))
        let routeStart = try XCTUnwrap(runner.range(
            of: "    private func routeSolveToOwningFrame(",
            range: segmentStart.upperBound..<runner.endIndex))
        let segment = runner[segmentStart.lowerBound..<routeStart.lowerBound]
        XCTAssertTrue(segment.contains("token: UInt64"))
        XCTAssertTrue(segment.contains("engineLease: NimbleEngine.OfflinePolicyLease"))
        XCTAssertTrue(segment.contains("enginePolicyLease == engineLease"))
        XCTAssertTrue(segment.contains("nimble.ownsOfflinePolicyLease(engineLease)"))

        let reset = try XCTUnwrap(segment.range(of: "nimble.resetRealtimeState("))
        let postReset = segment[reset.upperBound...]
        XCTAssertTrue(postReset.contains("runOwnership.isCurrent(token)"),
                      "a reset notification may synchronously start a successor")
        XCTAssertTrue(postReset.contains("enginePolicyLease == engineLease"))

        XCTAssertTrue(runner.contains("offlinePolicyLease: engineLease"),
                      "offline scale/reset calls must present the captured lease")
        let engine = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let scaleStart = try XCTUnwrap(engine.range(of: "    func scaleModel("))
        let processStart = try XCTUnwrap(engine.range(
            of: "    func processFrame(",
            range: scaleStart.upperBound..<engine.endIndex))
        let scale = engine[scaleStart.lowerBound..<processStart.lowerBound]
        XCTAssertTrue(scale.contains("offlinePolicyLease: OfflinePolicyLease? = nil"))
        XCTAssertTrue(scale.contains("permitsOfflinePolicyMutation(offlinePolicyLease)"))
        XCTAssertTrue(engine.contains("struct ModelScaleRecipe: Equatable, Sendable"),
                      "live calibration must be retained as value-only Swift state")
        XCTAssertTrue(scale.contains("let recipe = ModelScaleRecipe("))
        let scaleQueue = try XCTUnwrap(scale.range(of: "solverQueue.async"))
        let nativeScale = try XCTUnwrap(scale.range(
            of: "let succeeded = self.bridge.scaleModel(",
            range: scaleQueue.upperBound..<scale.endIndex))
        let usability = try XCTUnwrap(scale.range(
            of: "self.modelScaleIsUsable = succeeded",
            range: nativeScale.upperBound..<scale.endIndex))
        let nativeSuccess = try XCTUnwrap(scale.range(
            of: "if succeeded",
            range: usability.upperBound..<scale.endIndex))
        let liveOnly = try XCTUnwrap(scale.range(
            of: "if offlinePolicyLease == nil",
            range: nativeSuccess.upperBound..<scale.endIndex))
        let retainRecipe = try XCTUnwrap(scale.range(
            of: "self.liveScaleRecipe = recipe",
            range: liveOnly.upperBound..<scale.endIndex))
        let scaleQueueEnd = try XCTUnwrap(scale.range(
            of: "\n        }\n        return true",
            range: retainRecipe.upperBound..<scale.endIndex))
        XCTAssertLessThan(scaleQueue.lowerBound, nativeScale.lowerBound,
                          "native geometry mutation must stay solver-queue confined")
        XCTAssertLessThan(nativeScale.lowerBound, usability.lowerBound)
        XCTAssertLessThan(usability.lowerBound, nativeSuccess.lowerBound,
                          "a failed native scale must fail the next queued frame closed")
        XCTAssertLessThan(nativeSuccess.lowerBound, liveOnly.lowerBound)
        XCTAssertLessThan(liveOnly.lowerBound, retainRecipe.lowerBound,
                          "only a successful live scale may replace the live recipe")
        XCTAssertLessThan(retainRecipe.lowerBound, scaleQueueEnd.lowerBound,
                          "recipe/usable state must stay inside the native queue block")

        let loadStart = try XCTUnwrap(engine.range(of: "    func loadBundledModel()"))
        let load = engine[loadStart.lowerBound..<scaleStart.lowerBound]
        let loadQueue = try XCTUnwrap(load.range(of: "solverQueue.async"))
        let loadSuccess = try XCTUnwrap(load.range(
            of: "if success {",
            range: loadQueue.upperBound..<load.endIndex))
        let invalidate = try XCTUnwrap(load.range(
            of: "self.liveScaleRecipe = nil",
            range: loadSuccess.upperBound..<load.endIndex))
        let mainPublish = try XCTUnwrap(load.range(
            of: "DispatchQueue.main.async",
            range: invalidate.upperBound..<load.endIndex))
        XCTAssertLessThan(loadQueue.lowerBound, loadSuccess.lowerBound)
        XCTAssertLessThan(loadSuccess.lowerBound, invalidate.lowerBound)
        XCTAssertLessThan(invalidate.lowerBound, mainPublish.lowerBound,
                          "only a successful replacement model invalidates its recipe")
    }

    func testEnginePublishesAndResetsResultStateAsOneNonreentrantBatch() throws {
        let source = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let resultFields = [
            "lastIKResult", "lastIDResult", "lastMuscleResult",
            "displayMuscleResult", "ikSolveTimeMs", "idSolveTimeMs",
            "muscleSolveTimeMs", "ikMarkerResidualMeters", "maxTorquePerKg",
            "leftFootLoadFraction", "rightFootLoadFraction",
            "rootResidualPerKg", "groundHeightY", "lastSolve",
            "dynamicsAvailability"
        ]
        for field in resultFields {
            XCTAssertFalse(source.contains("@Published private(set) var \(field)"),
                           "\(field) would synchronously reenter a partial state transaction")
        }

        let publishStart = try XCTUnwrap(source.range(
            of: "    private func publishResults("))
        let completionStart = try XCTUnwrap(source.range(
            of: "    private func completeFrame(",
            range: publishStart.upperBound..<source.endIndex))
        let publish = source[publishStart.lowerBound..<completionStart.lowerBound]
        let lastWrite = try XCTUnwrap(publish.range(of: "self.groundHeightY = groundY"))
        let terminal = try XCTUnwrap(publish.range(
            of: "self.finishFrameOnMain(receipt: receipt, status: .published)",
            range: lastWrite.upperBound..<publish.endIndex))
        let notify = try XCTUnwrap(publish.range(
            of: "self.objectWillChange.send()",
            range: terminal.upperBound..<publish.endIndex))
        XCTAssertLessThan(lastWrite.lowerBound, terminal.lowerBound)
        XCTAssertLessThan(terminal.lowerBound, notify.lowerBound)

        let resetStart = try XCTUnwrap(source.range(
            of: "    private func resetRealtimeState("))
        let sessionStart = try XCTUnwrap(source.range(
            of: "    func resetSessionState(",
            range: resetStart.upperBound..<source.endIndex))
        let reset = source[resetStart.lowerBound..<sessionStart.lowerBound]
        XCTAssertTrue(reset.contains("resetsGroundHeight: Bool"))
        XCTAssertTrue(reset.contains(
            "restoresLiveModelScale: Bool = false"))
        let resetQueue = try XCTUnwrap(reset.range(of: "solverQueue.async"))
        let scaleRestore = try XCTUnwrap(reset.range(
            of: "if restoresLiveModelScale",
            range: resetQueue.upperBound..<reset.endIndex))
        XCTAssertTrue(reset.contains(
            "if restoresLiveModelScale, self.bridge.isModelLoaded"),
            "cancel before model load has no geometry to restore")
        let filterReset = try XCTUnwrap(reset.range(
            of: "self.dofFilters.removeAll",
            range: scaleRestore.upperBound..<reset.endIndex))
        let resetQueueEnd = try XCTUnwrap(reset.range(
            of: "\n        }\n\n        lastDisplayMuscleTimestamp",
            range: filterReset.upperBound..<reset.endIndex))
        XCTAssertLessThan(resetQueue.lowerBound, scaleRestore.lowerBound,
                          "geometry restoration must stay solver-queue confined")
        XCTAssertLessThan(scaleRestore.lowerBound, filterReset.lowerBound,
                          "the FIFO block must restore geometry before solver histories")
        XCTAssertLessThan(filterReset.lowerBound, resetQueueEnd.lowerBound)
        XCTAssertTrue(reset.contains("if let recipe = self.liveScaleRecipe"))
        XCTAssertTrue(reset.contains("bridge.restoreLoadedModelBodyScales()"))
        XCTAssertTrue(reset.contains("bridge.scaleModel("))
        XCTAssertTrue(reset.contains("self.liveScaleRecipe = nil"),
                      "a failed recipe must be discarded before default fallback")
        XCTAssertTrue(reset.contains("self.modelScaleIsUsable = restored"))
        let resetNotify = try XCTUnwrap(reset.range(of: "objectWillChange.send()"))
        let resetLastWrite = try XCTUnwrap(reset.range(of: "rootResidualPerKg = 0"))
        XCTAssertLessThan(resetLastWrite.lowerBound, resetNotify.lowerBound)
        XCTAssertTrue(reset.contains("lastIDResult = nil"))
        XCTAssertTrue(reset.contains("lastMuscleResult = nil"))
        XCTAssertTrue(reset.contains("displayMuscleResult = nil"))

        let supersedeStart = try XCTUnwrap(source.range(
            of: "    @discardableResult\n    func supersedeFrame(",
            range: completionStart.upperBound..<source.endIndex))
        let completion = source[completionStart.lowerBound..<supersedeStart.lowerBound]
        XCTAssertTrue(completion.contains("if status == .failed"))
        XCTAssertTrue(completion.contains("resetRealtimeState("),
                      "a post-transition IK failure must synchronously clear old dynamics")

        let processStart = try XCTUnwrap(source.range(of: "    func processFrame("))
        let process = source[processStart.lowerBound..<resetStart.lowerBound]
        let referenceFence = try XCTUnwrap(process.range(
            of: "self.prepareForDynamicsReference(dynamicsReference)"))
        let solveIK = try XCTUnwrap(process.range(
            of: "self.bridge.solveIK(",
            range: referenceFence.upperBound..<process.endIndex))
        XCTAssertLessThan(referenceFence.lowerBound, solveIK.lowerBound,
                          "reference changes must reset warm-start state before IK")

        let referenceResetStart = try XCTUnwrap(source.range(
            of: "    private func prepareForDynamicsReference("))
        let referenceReset = source[referenceResetStart.lowerBound..<processStart.lowerBound]
        XCTAssertTrue(referenceReset.contains("dofFilters.removeAll"))
        XCTAssertTrue(referenceReset.contains("holdDetector.reset()"))
        XCTAssertTrue(referenceReset.contains("bridge.resetSessionState()"),
                      "coordinate changes must discard the old ground and IK warm start")
        XCTAssertTrue(referenceReset.contains("muscleSolver.resetSessionState()"))
        XCTAssertTrue(referenceReset.contains("activationFilters.removeAll"))
        XCTAssertTrue(referenceReset.contains(
            "dynamicsReferenceWarmupInvalidationPending = true"),
            "the first post-transition publication must erase the old overlay")

        let warmupStart = try XCTUnwrap(process.range(of: "            guard sgWarmedUp else"))
        let warmupEnd = try XCTUnwrap(process.range(
            of: "            // Smoothed IK output",
            range: warmupStart.upperBound..<process.endIndex))
        let warmup = process[warmupStart.lowerBound..<warmupEnd.lowerBound]
        XCTAssertTrue(warmup.contains("dynamicsReferenceWarmupInvalidationPending"))
        XCTAssertTrue(warmup.contains(".referenceTransitionWarmup"))
        XCTAssertFalse(warmup.contains(".rootTrajectoryUnavailable"),
                       "a qualified new reference must not be mislabeled unqualified")
        XCTAssertFalse(warmup.contains(".gravityReferenceUnavailable"),
                       "transition warm-up is distinct from a missing gravity reference")

        let sessionEnd = try XCTUnwrap(source.range(
            of: "    func resetAnalysisPassStatePreservingGround(",
            range: sessionStart.upperBound..<source.endIndex))
        let session = source[sessionStart.lowerBound..<sessionEnd.lowerBound]
        XCTAssertTrue(session.contains("resetsGroundHeight: true"))
        XCTAssertFalse(session.contains("restoresLiveModelScale: true"),
                       "ordinary session boundaries preserve the live subject scale")
        XCTAssertFalse(session.contains("groundHeightY = 0"),
                       "ground reset before the batch would synchronously reenter mid-transaction")
    }

    func testRunnerCleanupRetiresLocalOwnershipBeforeEngineRelease() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let runInternalStart = try XCTUnwrap(source.range(
            of: "    private func runInternal("))
        let gaitStart = try XCTUnwrap(source.range(
            of: "    // MARK: - Gait pass",
            range: runInternalStart.upperBound..<source.endIndex))
        let runInternal = source[runInternalStart.lowerBound..<gaitStart.lowerBound]

        let complete = try XCTUnwrap(runInternal.range(
            of: "runOwnership.complete(token)"))
        let release = try XCTUnwrap(runInternal.range(
            of: "nimble.releaseOfflinePolicyLease(engineLease)"))
        XCTAssertLessThan(complete.lowerBound, release.lowerBound)
        XCTAssertTrue(runInternal.contains("enginePolicyLease == engineLease"))
        let cleanupPrefix = runInternal[complete.lowerBound..<release.lowerBound]
        XCTAssertTrue(cleanupPrefix.contains("enginePolicyLease = nil"))
        XCTAssertTrue(cleanupPrefix.contains("runTask = nil"))
    }

    func testEngineGlobalLeaseRejectsLiveAndStaleRunnerAdmissions() throws {
        let engine = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let acquire = try XCTUnwrap(engine.range(
            of: "    func acquireOfflinePolicyLease()"))
        let release = try XCTUnwrap(engine.range(
            of: "    func releaseOfflinePolicyLease(",
            range: acquire.upperBound..<engine.endIndex))
        let process = try XCTUnwrap(engine.range(
            of: "    func processFrame(",
            range: release.upperBound..<engine.endIndex))
        let leaseCode = engine[acquire.lowerBound..<process.lowerBound]
        XCTAssertTrue(leaseCode.contains("activeOfflinePolicyLease = lease"))
        XCTAssertTrue(leaseCode.contains(
            "guard activeOfflinePolicyLease == lease else { return false }"))
        let releaseCode = engine[release.lowerBound..<process.lowerBound]
        let restorePolicy = try XCTUnwrap(releaseCode.range(
            of: "cameraDynamicsAuthorization = .unrestricted"))
        let notifyReset = try XCTUnwrap(releaseCode.range(
            of: "resetRealtimeState(",
            range: restorePolicy.upperBound..<releaseCode.endIndex))
        XCTAssertLessThan(restorePolicy.lowerBound, notifyReset.lowerBound,
                          "a reentrant successor cannot be overwritten after reset notification")
        XCTAssertTrue(releaseCode.contains("restoresLiveModelScale: true"))
        XCTAssertTrue(releaseCode.contains("resetsBridgeSession: true"))
        XCTAssertTrue(releaseCode.contains("resetsMuscleSession: true"))
        XCTAssertTrue(releaseCode.contains("resetsGroundHeight: true"))

        let resetStart = try XCTUnwrap(engine.range(
            of: "    func resetRealtimeState(",
            range: process.upperBound..<engine.endIndex))
        let admission = engine[process.lowerBound..<resetStart.lowerBound]
        XCTAssertTrue(admission.contains(
            "switch (activeOfflinePolicyLease, offlinePolicyLease)"))
        XCTAssertTrue(admission.contains("where active == supplied"))
        XCTAssertTrue(admission.contains("return .rejected"))
        XCTAssertTrue(admission.contains("guard self.modelScaleIsUsable else"),
                      "an invariant-level restore failure must fail frame solving closed")

        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        XCTAssertTrue(runner.contains(
            "offlinePolicyLease: enginePolicyLease"))
        XCTAssertTrue(runner.contains(
            "nimble.ownsOfflinePolicyLease(enginePolicyLease)"))
    }

    func testEngineGlobalLeaseAlsoOwnsResetMutations() throws {
        let engine = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let realtime = try XCTUnwrap(engine.range(
            of: "    func resetRealtimeState("))
        let session = try XCTUnwrap(engine.range(
            of: "    func resetSessionState(",
            range: realtime.upperBound..<engine.endIndex))
        let resetAPI = engine[realtime.lowerBound..<session.lowerBound]
        XCTAssertTrue(resetAPI.contains(
            "offlinePolicyLease: OfflinePolicyLease? = nil"))
        XCTAssertTrue(resetAPI.contains(
            "guard permitsOfflinePolicyMutation(offlinePolicyLease) else"))

        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let compactRunner = runner.filter { !$0.isWhitespace }
        XCTAssertTrue(compactRunner.contains(
            "resetSessionState(offlinePolicyLease:engineLease)"))
        XCTAssertTrue(compactRunner.contains(
            "resetRealtimeState(offlinePolicyLease:enginePolicyLease)"))
        XCTAssertTrue(compactRunner.contains(
            "resetAnalysisPassStatePreservingGround(offlinePolicyLease:enginePolicyLease)"))

        let content = try Self.source("BioMotion/App/ContentView.swift")
        let trackingStart = try XCTUnwrap(content.range(
            of: ".onChange(of: bodyTracking.isTracking)"))
        let offlineSheet = try XCTUnwrap(content.range(
            of: ".onChange(of: showOfflineImport)",
            range: trackingStart.upperBound..<content.endIndex))
        let trackingHandler = content[trackingStart.lowerBound..<offlineSheet.lowerBound]
        XCTAssertTrue(trackingHandler.contains("guard !showOfflineImport else { return }"),
                      "a queued AR tracking-loss callback must not reset an offline run")
    }

    @MainActor
    func testOfflineLeaseRejectsOwnerlessResetButAcceptsItsExactOwner() {
        let engine = NimbleEngine()
        let lease = engine.acquireOfflinePolicyLease()

        XCTAssertFalse(engine.resetRealtimeState())
        XCTAssertFalse(engine.resetSessionState())
        XCTAssertFalse(engine.resetAnalysisPassStatePreservingGround())
        XCTAssertTrue(engine.resetRealtimeState(offlinePolicyLease: lease))
        XCTAssertTrue(engine.resetSessionState(offlinePolicyLease: lease))
        XCTAssertTrue(engine.resetAnalysisPassStatePreservingGround(
            offlinePolicyLease: lease
        ))
        XCTAssertTrue(engine.releaseOfflinePolicyLease(lease))
    }

    func testWaiterKeepsTimeoutFailureAndAdmissionReasonsDistinct() throws {
        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let submitStart = try XCTUnwrap(runner.range(
            of: "    private func submitAndWait("))
        let waiterStart = try XCTUnwrap(runner.range(
            of: "enum NimbleFrameWaiter",
            range: submitStart.upperBound..<runner.endIndex))
        let submit = runner[submitStart.lowerBound..<waiterStart.lowerBound]
        XCTAssertTrue(submit.contains("case .failed:"))
        XCTAssertTrue(submit.contains(".failure(.solveFailed)"))
        XCTAssertTrue(submit.contains("case .timedOut:"))
        XCTAssertTrue(submit.contains(".failure(.timedOut)"))
        XCTAssertTrue(submit.contains("case .rejected:"))
        XCTAssertTrue(submit.contains(".failure(.admissionRejected)"))
        XCTAssertTrue(submit.contains(".failure(.busy)"))
        XCTAssertFalse(submit.contains("case .failed, .superseded, .rejected:"),
                       "terminal causes must not collapse into a fake timeout")

        let waiter = runner[waiterStart.lowerBound...]
        XCTAssertTrue(waiter.contains("case timedOut"))
        XCTAssertTrue(waiter.contains("finish(.timedOut)"))
        XCTAssertTrue(waiter.contains("finish(.superseded)"),
                      "external reset/cancellation remains distinct from timeout")
    }

    func testReceiptWaiterDoesNotUseCoarseObjectChangeOrSettleDelay() throws {
        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let waiterStart = try XCTUnwrap(runner.range(of: "enum NimbleFrameWaiter"))
        let waiter = runner[waiterStart.lowerBound...]
        XCTAssertFalse(waiter.contains("objectWillChange"))
        XCTAssertFalse(waiter.contains("publishSettleDelay"))
        XCTAssertTrue(waiter.contains("frameCompletionPublisher"))
        XCTAssertTrue(waiter.contains(
            "completion.receipt == self.expectedReceipt"))
        XCTAssertTrue(waiter.contains("withTaskCancellationHandler"))

        let engine = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let processStart = try XCTUnwrap(engine.range(of: "    func processFrame("))
        let resetStart = try XCTUnwrap(engine.range(
            of: "    func resetRealtimeState(",
            range: processStart.upperBound..<engine.endIndex))
        let process = engine[processStart.lowerBound..<resetStart.lowerBound]
        XCTAssertTrue(process.contains("-> FrameSubmission"))
        XCTAssertTrue(process.contains("FrameReceipt("))
        XCTAssertTrue(process.contains("completeFrame(receipt"))
    }

    func testTimeoutAtomicallySupersedesExactReceiptAndPaddingStopsWithoutFalsePushCount() throws {
        let source = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        let submitStart = try XCTUnwrap(source.range(of: "    private func submitAndWait("))
        let waiterStart = try XCTUnwrap(source.range(
            of: "enum NimbleFrameWaiter",
            range: submitStart.upperBound..<source.endIndex))
        let submit = source[submitStart.lowerBound..<waiterStart.lowerBound]
        XCTAssertFalse(submit.contains("nimble.resetRealtimeState()"),
                       "caller must not broadly reset a possible successor")

        let exactFenceStart = try XCTUnwrap(source.range(
            of: "        private func supersedeAndFinish(",
            range: waiterStart.upperBound..<source.endIndex))
        let finishStart = try XCTUnwrap(source.range(
            of: "        func finish(_ outcome: Outcome)",
            range: exactFenceStart.upperBound..<source.endIndex))
        let exactFenceBody = source[exactFenceStart.lowerBound..<finishStart.lowerBound]
        let stopObserving = try XCTUnwrap(exactFenceBody.range(
            of: "cancellable?.cancel()"))
        let exactFence = try XCTUnwrap(exactFenceBody.range(
            of: "engine?.supersedeFrame(expectedReceipt)"))
        let resume = try XCTUnwrap(exactFenceBody.range(of: "finish(outcome)"))
        XCTAssertLessThan(stopObserving.lowerBound, exactFence.lowerBound,
                          "local timeout must not let synchronous .superseded steal its reason")
        XCTAssertLessThan(exactFence.lowerBound, resume.lowerBound,
                          "generation must be fenced before timeout resumes")
        XCTAssertTrue(source.contains("supersedeAndFinish(.timedOut)"))
        XCTAssertTrue(source.contains("supersedeAndFinish(.superseded)"))
        XCTAssertTrue(source.contains("state.cancel()"),
                      "task cancellation must use the same exact fence")

        let primeStart = try XCTUnwrap(source.range(of: "    private func primeFilterHead("))
        let tailStart = try XCTUnwrap(source.range(
            of: "    private func padFilterTail(",
            range: primeStart.upperBound..<source.endIndex))
        let etaStart = try XCTUnwrap(source.range(
            of: "    private func eta(",
            range: tailStart.upperBound..<source.endIndex))
        let prime = source[primeStart.lowerBound..<tailStart.lowerBound]
        let tail = source[tailStart.lowerBound..<etaStart.lowerBound]
        let primeGuard = try XCTUnwrap(prime.range(
            of: "guard case .success = submission else { return false }"))
        let primeIncrement = try XCTUnwrap(prime.range(of: "totalPushes += 1"))
        XCTAssertLessThan(primeGuard.lowerBound, primeIncrement.lowerBound)
        let tailGuard = try XCTUnwrap(tail.range(
            of: "guard case .success(let receipt) = submission else { return false }"))
        let tailIncrement = try XCTUnwrap(tail.range(of: "totalPushes += 1"))
        XCTAssertLessThan(tailGuard.lowerBound, tailIncrement.lowerBound)
        XCTAssertTrue(source.contains("guard primed else { return }"),
                      "private gait replay must stop after failed head context")
        XCTAssertTrue(source.contains("guard padded else { return }"),
                      "private gait replay must stop after failed tail context")
        XCTAssertTrue(source.contains("case .published(let receipt):"))
        XCTAssertTrue(source.contains("nimble.lastSolveReceipt == receipt"),
                      "global solve snapshots must match the exact completion")
    }

    func testResetRevokesPublicationButKeepsPhysicalSolverOccupiedUntilTerminalPath() throws {
        let source = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        XCTAssertTrue(source.contains(
            "private var solverOccupancyReceipt: FrameReceipt?"))

        let finishStart = try XCTUnwrap(source.range(
            of: "    private func finishFrameOnMain("))
        let resetStart = try XCTUnwrap(source.range(
            of: "    func resetRealtimeState(",
            range: finishStart.upperBound..<source.endIndex))
        let finish = source[finishStart.lowerBound..<resetStart.lowerBound]
        let occupancyGuard = try XCTUnwrap(finish.range(
            of: "guard solverOccupancyReceipt == receipt else { return }"))
        let occupancyClear = try XCTUnwrap(finish.range(
            of: "solverOccupancyReceipt = nil"))
        let releaseBackpressure = try XCTUnwrap(finish.range(
            of: "isFrameInFlight = false"))
        let publicationGuard = try XCTUnwrap(finish.range(
            of: "guard activeFrameReceipt == receipt else { return }"))
        XCTAssertLessThan(occupancyGuard.lowerBound, occupancyClear.lowerBound)
        XCTAssertLessThan(occupancyClear.lowerBound, releaseBackpressure.lowerBound)
        XCTAssertLessThan(releaseBackpressure.lowerBound,
                          publicationGuard.lowerBound)

        let sessionResetStart = try XCTUnwrap(source.range(
            of: "    func resetSessionState(",
            range: resetStart.upperBound..<source.endIndex))
        let reset = source[resetStart.lowerBound..<sessionResetStart.lowerBound]
        XCTAssertTrue(reset.contains("activeFrameReceipt = nil"),
                      "reset must revoke the old publication lease immediately")
        XCTAssertFalse(reset.contains("solverOccupancyReceipt = nil"),
                       "reset cannot pretend non-cancellable solver work ended")
        XCTAssertFalse(reset.contains("isFrameInFlight = false"),
                       "a timeout must not enqueue B/C behind a still-running A")
    }

    func testImportCloseCancelsBeforeDismissAndNavigationBackstopPreservesPlayback() throws {
        let source = try Self.source("BioMotion/Offline/OfflineImportView.swift")
        let closeStart = try XCTUnwrap(source.range(of: "    private func close()"))
        let close = source[closeStart.lowerBound...]
        let cancel = try XCTUnwrap(close.range(of: "runner.cancel()"))
        let dismiss = try XCTUnwrap(close.range(of: "onDismiss()"))
        XCTAssertLessThan(cancel.lowerBound, dismiss.lowerBound)
        XCTAssertTrue(source.contains(".interactiveDismissDisabled(isRunning)"))
        XCTAssertTrue(source.contains("guard !showPlayback else { return }"))
    }

    func testEngineChecksCapturedCameraAuthorizationBeforeInverseDynamics() throws {
        let source = try Self.source("BioMotion/Nimble/NimbleEngine.swift")
        let processStart = try XCTUnwrap(source.range(of: "    func processFrame("))
        let processEnd = try XCTUnwrap(source.range(
            of: "    static let rootVerticalDOFName",
            range: processStart.upperBound..<source.endIndex))
        let process = source[processStart.lowerBound..<processEnd.lowerBound]
        let finiteFence = try XCTUnwrap(process.range(
            of: "guard joint.worldPosition.x.isFinite"))
        let receiptCreation = try XCTUnwrap(process.range(of: "let frameGeneration"))
        XCTAssertLessThan(finiteFence.lowerBound, receiptCreation.lowerBound,
                          "invalid marker numbers must be rejected before native IK admission")
        let capture = try XCTUnwrap(process.range(
            of: "let cameraAuthorization = cameraDynamicsAuthorization"))
        let referenceCapture = try XCTUnwrap(process.range(
            of: "let dynamicsReference = frame.dynamicsReference"))
        let solverQueue = try XCTUnwrap(process.range(of: "solverQueue.async"))
        XCTAssertLessThan(capture.lowerBound, solverQueue.lowerBound)
        XCTAssertLessThan(referenceCapture.lowerBound, solverQueue.lowerBound)

        let preflight = try XCTUnwrap(process.range(
            of: "Self.dynamicsPreflightAvailability("))
        let inverseDynamics = try XCTUnwrap(process.range(of: "self.bridge.solveIDGRF("))
        XCTAssertLessThan(preflight.lowerBound, inverseDynamics.lowerBound)
        XCTAssertEqual(process.components(
            separatedBy: "dynamicsReference: dynamicsReference").count - 1, 2,
                       "gait and ordinary ID paths must use the captured reference")

        let runner = try Self.source("BioMotion/Offline/OfflineSessionRunner.swift")
        XCTAssertEqual(runner.components(
            separatedBy: "dynamicsReference: bodyFrame.dynamicsReference").count - 1, 2,
                       "head and tail SG padding must preserve provenance")

        let tracking = try Self.source("BioMotion/ARKit/BodyTrackingSession.swift")
        XCTAssertTrue(tracking.contains("config.worldAlignment = .gravity"))
        XCTAssertTrue(tracking.contains("dynamicsReference: .liveARKit"))
    }

    func testPermanentCapabilityBannerSuppressesCameraQualityAdvice() throws {
        let source = try Self.source("BioMotion/Offline/OfflinePlaybackView.swift")
        let contact = try XCTUnwrap(source.range(
            of: "if resultStore.hasValidatedFootContactSupport == false"))
        let camera = try XCTUnwrap(source.range(
            of: "if resultStore.hasValidatedFootContactSupport != false,"))
        XCTAssertLessThan(contact.lowerBound, camera.lowerBound)
        XCTAssertTrue(source[camera.lowerBound...].hasPrefix(
            "if resultStore.hasValidatedFootContactSupport != false,"))
    }

    private static let evidence = CameraMotionEvidence(
        analyzedDurationSeconds: 1,
        derivativeWindowSeconds: 0.25,
        peakNormalizedTranslation: 0.004,
        translationStaticUpperBound: 0.01,
        translationMovingLowerBound: 0.02,
        peakRotationRadians: 0.004,
        rotationStaticUpperBoundRadians: 0.01,
        rotationMovingLowerBoundRadians: 0.02,
        peakScaleFraction: 0.003,
        scaleStaticUpperBound: 0.01,
        scaleMovingLowerBound: 0.02,
        calibrationProfileID: "tripod-controls-v1",
        analysisFrameWidthPixels: 1280,
        analysisFrameHeightPixels: 720,
        maximumAnalysisPixelCount: 2_073_600,
        measuredIntervals: 30,
        sampledFrames: 31
    )

    private static let uncalibratedEvidence = CameraMotionEvidence(
        analyzedDurationSeconds: evidence.analyzedDurationSeconds,
        derivativeWindowSeconds: evidence.derivativeWindowSeconds,
        peakNormalizedTranslation: evidence.peakNormalizedTranslation,
        translationStaticUpperBound: evidence.translationStaticUpperBound,
        translationMovingLowerBound: evidence.translationMovingLowerBound,
        peakRotationRadians: evidence.peakRotationRadians,
        rotationStaticUpperBoundRadians: evidence.rotationStaticUpperBoundRadians,
        rotationMovingLowerBoundRadians: evidence.rotationMovingLowerBoundRadians,
        peakScaleFraction: evidence.peakScaleFraction,
        scaleStaticUpperBound: evidence.scaleStaticUpperBound,
        scaleMovingLowerBound: evidence.scaleMovingLowerBound,
        calibrationProfileID: nil,
        analysisFrameWidthPixels: evidence.analysisFrameWidthPixels,
        analysisFrameHeightPixels: evidence.analysisFrameHeightPixels,
        maximumAnalysisPixelCount: evidence.maximumAnalysisPixelCount,
        measuredIntervals: evidence.measuredIntervals,
        sampledFrames: evidence.sampledFrames
    )

    private static func availableFrame(
        id: Int = 0,
        isStatic: Bool,
        motionState: OfflineResultStore.MotionState? = nil,
        includesMuscle: Bool = true
    ) -> OfflineResultStore.FrameResult {
        let timestamp = Double(id)
        let body = BodyFrame(
            timestamp: timestamp,
            frameNumber: id,
            joints: [TrackedJoint(id: "hips_joint", name: "Pelvis",
                                  worldPosition: .zero, isTracked: true)],
            dynamicsReference: .dynamicsQualifiedWorld
        )
        return OfflineResultStore.FrameResult(
            id: id,
            sourceImage: UIImage(),
            timestamp: timestamp,
            status: .success,
            usedFallbackBBox: false,
            camT: nil,
            modelChecksums: nil,
            bodyFrame: body,
            ikResult: NimbleEngine.IKOutput(
                jointAngles: ["generation": Double(id)],
                markerRMSMeters: 0,
                ikLossSquaredMeters: 0,
                timestamp: timestamp
            ),
            idResult: NimbleEngine.IDOutput(
                jointTorques: ["generation": Double(id)],
                timestamp: timestamp
            ),
            muscleResult: includesMuscle
                ? NimbleEngine.MuscleOutput(
                    activations: ["generation": Double(id)],
                    forces: ["generation": Double(id)],
                    converged: true,
                    timestamp: timestamp
                )
                : nil,
            dynamicsAvailability: .available,
            isStaticHoldEstimate: isStatic,
            motionState: motionState ?? (isStatic
                ? .measured(verdict: .hold,
                            peakSpeedMetersPerSecond: 0,
                            windowSeconds: 0.25,
                            noiseFloorMetersPerSecond: 0.001)
                : .gait(verdict: .gaitStance, outcome: nil))
        )
    }

    private static func source(_ relativePath: String) throws -> String {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        return try String(
            contentsOf: repositoryRoot.appendingPathComponent(relativePath),
            encoding: .utf8
        )
    }
}
