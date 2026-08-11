import simd
import Vision
import XCTest

@testable import BioMotion

/// Product-level camera-reference policy, independent of Vision and video I/O.
/// The adapter is allowed to estimate motion; only this reducer decides whether
/// that evidence is complete enough to license a dynamics window.
final class CameraMotionReducerTests: XCTestCase {

    func testProductionCameraBandsRemainUncalibrated() {
        XCTAssertEqual(CameraMotionVideoAnalyzer.calibrationAdapterRevision, 3)
        XCTAssertNil(CameraMotionVideoAnalyzer.Configuration.production.calibrationProfileID)
        XCTAssertNil(
            CameraMotionVideoAnalyzer.Configuration.production.calibrationFrameSizePixels
        )
        XCTAssertEqual(
            CameraMotionVideoAnalyzer.Configuration.production.maximumAnalysisPixelCount,
            1_920 * 1_080
        )
        XCTAssertFalse(CameraMotionVideoAnalyzer().isVersionedCalibrationReady)
    }

    func testVisionRequestRevisionsArePinnedSupportedAndAssignedBeforeExecution() throws {
        XCTAssertEqual(
            CameraMotionVideoAnalyzer.humanRectanglesRequestRevision,
            VNDetectHumanRectanglesRequestRevision2
        )
        XCTAssertEqual(
            CameraMotionVideoAnalyzer.translationalRegistrationRequestRevision,
            VNTranslationalImageRegistrationRequestRevision1
        )
        XCTAssertTrue(
            CameraMotionVideoAnalyzer.pinnedVisionRequestRevisionsAreSupported
        )

        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let source = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Offline/CameraMotionVideoAnalyzer.swift"
            ),
            encoding: .utf8
        )

        let registrationConstruction = try XCTUnwrap(source.range(
            of: "let request = VNTranslationalImageRegistrationRequest("
        ))
        let registrationRevision = try XCTUnwrap(source.range(
            of: "request.revision = Self.translationalRegistrationRequestRevision"
        ))
        let registrationExecution = try XCTUnwrap(source.range(
            of: "try handler.perform(requests)"
        ))
        XCTAssertLessThan(
            registrationConstruction.lowerBound,
            registrationRevision.lowerBound
        )
        XCTAssertLessThan(
            registrationRevision.lowerBound,
            registrationExecution.lowerBound
        )

        let humanConstruction = try XCTUnwrap(source.range(
            of: "let request = VNDetectHumanRectanglesRequest()"
        ))
        let humanRevision = try XCTUnwrap(source.range(
            of: "request.revision = Self.humanRectanglesRequestRevision"
        ))
        let humanExecution = try XCTUnwrap(source.range(
            of: "try handler.perform([request])"
        ))
        XCTAssertLessThan(humanConstruction.lowerBound, humanRevision.lowerBound)
        XCTAssertLessThan(humanRevision.lowerBound, humanExecution.lowerBound)
    }

    func testVisionRevisionFingerprintDriftFailsBothSafetyBoundaries() {
        let profileID = "tripod-controls-v1"
        let frameSize = SIMD2(1_280, 720)
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintHumanRectanglesRequestRevision:
                CameraMotionVideoAnalyzer.humanRectanglesRequestRevision + 1
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintTranslationalRegistrationRequestRevision:
                CameraMotionVideoAnalyzer
                    .translationalRegistrationRequestRevision + 1
        )))

        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        for refused in [
            reducerConfiguration(
                fingerprintHumanRectanglesRequestRevision:
                    CameraMotionVideoAnalyzer.humanRectanglesRequestRevision + 1
            ),
            reducerConfiguration(
                fingerprintTranslationalRegistrationRequestRevision:
                    CameraMotionVideoAnalyzer
                        .translationalRegistrationRequestRevision + 1
            ),
        ] {
            guard case .calibrationRequired = CameraMotionReducer.reduce(
                observations: measurements,
                configuration: refused
            ) else {
                return XCTFail(
                    "both reducer-side Vision revisions must match the calibrated adapter"
                )
            }
        }
    }

    func testAliasScreenFingerprintDriftFailsBothSafetyBoundaries() {
        let profileID = "tripod-controls-v1"
        let frameSize = SIMD2(1_280, 720)
        let readinessDrift = [
            videoConfiguration(
                profileID: profileID,
                frameSizePixels: frameSize,
                fingerprintRegistrationAliasMinimumOverlapPairCount:
                    CameraRegistrationPeakAnalyzer
                        .aliasMinimumOverlapPairCount + 1
            ),
            videoConfiguration(
                profileID: profileID,
                frameSizePixels: frameSize,
                fingerprintRegistrationAliasSharedDomainSideSamples:
                    CameraRegistrationPeakAnalyzer
                        .aliasSharedDomainSideSamples + 1
            ),
            videoConfiguration(
                profileID: profileID,
                frameSizePixels: frameSize,
                fingerprintRegistrationAliasTailPairCount:
                    CameraRegistrationPeakAnalyzer.aliasTailPairCount - 1
            ),
        ]
        XCTAssertTrue(readinessDrift.allSatisfy { !isReady($0) })

        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        let reducerDrift = [
            reducerConfiguration(
                fingerprintRegistrationAliasMinimumOverlapPairCount:
                    CameraRegistrationPeakAnalyzer
                        .aliasMinimumOverlapPairCount + 1
            ),
            reducerConfiguration(
                fingerprintRegistrationAliasSharedDomainSideSamples:
                    CameraRegistrationPeakAnalyzer
                        .aliasSharedDomainSideSamples + 1
            ),
            reducerConfiguration(
                fingerprintRegistrationAliasTailPairCount:
                    CameraRegistrationPeakAnalyzer.aliasTailPairCount - 1
            ),
        ]
        for configuration in reducerDrift {
            guard case .calibrationRequired = CameraMotionReducer.reduce(
                observations: measurements,
                configuration: configuration
            ) else {
                return XCTFail("every alias-screen parameter must be fingerprinted")
            }
        }
    }

    func testCalibrationReadinessDefaultsClosedAndRequiresAValidVersionedFrameDomain() {
        XCTAssertFalse(DefaultCalibrationReadinessAnalyzer().isVersionedCalibrationReady)

        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: nil,
            frameSizePixels: SIMD2(1_280, 720)
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "   ",
            frameSizePixels: SIMD2(1_280, 720)
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: nil
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: SIMD2(0, 720)
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: SIMD2(1_279, 719)
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: SIMD2(2_000, 2_000)
        )).isVersionedCalibrationReady)
        XCTAssertFalse(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: SIMD2(2, 2)
        )).isVersionedCalibrationReady)

        XCTAssertTrue(CameraMotionVideoAnalyzer(configuration: videoConfiguration(
            profileID: "tripod-controls-v1",
            frameSizePixels: SIMD2(1_280, 720)
        )).isVersionedCalibrationReady)
    }

    func testCalibrationReadinessRejectsUnrunnableAppearanceDomains() {
        let profileID = "tripod-controls-v1"
        let frameSize = SIMD2(1_280, 720)
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            appearanceBoxAverageSizePixels: 0
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            appearanceBoxAverageSizePixels: 8,
            appearanceBoxAverageSpacingPixels: 4
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            registrationCorrelationSampleSpacingPixels: 4_096
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            registrationPeakSearchRadiusPixels: 8,
            registrationPeakMinimumSeparationPixels: 16
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            appearanceBoxAverageSizePixels: 2_048,
            appearanceBoxAverageSpacingPixels: 2_048,
            registrationCorrelationSampleSpacingPixels: 2_048,
            registrationPeakSearchRadiusPixels: 2_048,
            registrationPeakMinimumSeparationPixels: 2_048
        )), "matching fingerprint values cannot make an impossible grid ready")
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            appearanceBoxAverageSizePixels: 1,
            appearanceBoxAverageSpacingPixels: 1,
            registrationCorrelationSampleSpacingPixels: 1,
            registrationPeakSearchRadiusPixels: 64,
            registrationPeakMinimumSeparationPixels: 1
        )), "a valid ratio set cannot exceed the fingerprinted pair-cost cap")
    }

    func testCalibrationReadinessBindsTheCompleteAdapterFingerprint() {
        let profileID = "tripod-controls-v1"
        let frameSize = SIMD2(1_280, 720)
        XCTAssertTrue(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            includesFingerprint: false
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintProfileID: "tripod-controls-v0"
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintFrameSizePixels: SIMD2(1_080, 720)
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintAdapterRevision:
                CameraMotionVideoAnalyzer.calibrationAdapterRevision + 1
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            provisionalStaticTranslation: 0.011,
            fingerprintStaticTranslation: 0.01
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            maximumNativeFrameGapSeconds: 0.074,
            fingerprintMaximumNativeFrameGapSeconds: 0.075
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            maximumAnalysisPixelCount: 1_500_000,
            fingerprintMaximumAnalysisPixelCount: 1_920 * 1_080
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            maximumAnalysisDimensionPixels: 4_096,
            fingerprintMaximumAnalysisDimensionPixels: 4_094
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            maximumAnalysisAspectRatio: 4,
            fingerprintMaximumAnalysisAspectRatio: 3.5
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            maximumRetainedPixelBufferBytes: 64 * 1_024 * 1_024,
            fingerprintMaximumRetainedPixelBufferBytes: 63 * 1_024 * 1_024
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintMaximumRetainedPixelBufferCount:
                CameraAnalysisBufferBudget.maximumRetainedBufferCount - 1
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintAppearanceBoxAverageSizePixels: 4
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintAppearanceBoxAverageSpacingPixels: 16
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintRegistrationCorrelationSampleSpacingPixels: 24
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintRegistrationPeakSearchRadiusPixels: 40
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintRegistrationPeakMinimumSeparationPixels: 24
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            fingerprintMaximumRegistrationCorrelationPairCountPerTile:
                CameraRegistrationPeakAnalyzer
                    .maximumCorrelationPairCountPerTile - 1
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            personBoxInflationFraction: 0.04,
            fingerprintPersonBoxInflationFraction: 0.03
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            minimumRegistrationQuality: 0.55,
            fingerprintMinimumRegistrationQuality: 0.5
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            minimumRegistrationPeakCorrelation: 0.55,
            fingerprintMinimumRegistrationPeakCorrelation: 0.5
        )))
        XCTAssertFalse(isReady(videoConfiguration(
            profileID: profileID,
            frameSizePixels: frameSize,
            minimumRegistrationUniqueness: 0.06,
            fingerprintMinimumRegistrationUniqueness: 0.05
        )))
    }

    func testCameraReferenceAnalysisAdmissionMatrixIsFailClosed() {
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.decide(
                isSingleFrame: true,
                hasValidatedFootContactSupport: true,
                isVersionedCalibrationReady: true
            ),
            .singleFrame
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.decide(
                isSingleFrame: false,
                hasValidatedFootContactSupport: nil,
                isVersionedCalibrationReady: true
            ),
            .contactUnavailable
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.decide(
                isSingleFrame: false,
                hasValidatedFootContactSupport: false,
                isVersionedCalibrationReady: true
            ),
            .contactUnavailable
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.decide(
                isSingleFrame: false,
                hasValidatedFootContactSupport: true,
                isVersionedCalibrationReady: false
            ),
            .calibrationUnavailable
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.decide(
                isSingleFrame: false,
                hasValidatedFootContactSupport: true,
                isVersionedCalibrationReady: true
            ),
            .analyze
        )

        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.singleFrame.resolvedState,
            .notRequiredForSingleFrame
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.contactUnavailable.resolvedState,
            .unmeasured
        )
        XCTAssertEqual(
            CameraReferenceAnalysisAdmission.calibrationUnavailable.resolvedState,
            .calibrationUnavailable
        )
        XCTAssertNil(CameraReferenceAnalysisAdmission.analyze.resolvedState)
    }

    func testCalibrationUnavailableStateIsExplicitAndDeniesBothSolveClasses() {
        let state = CameraReferenceState.calibrationUnavailable
        XCTAssertFalse(state.permitsStaticEquilibrium)
        XCTAssertFalse(state.permitsTemporalDynamics)
        XCTAssertEqual(state.bannerTitle, "Camera calibration is not installed")
        XCTAssertEqual(
            state.bannerDetail,
            "This build has no versioned camera-motion calibration profile. Pose, anatomy and contact timing remain available; re-filming cannot enable dynamics."
        )
    }

    func testCameraScanBudgetCapsRequestedDurationAndActualNativeSamples() {
        XCTAssertNil(CameraMotionScanBudgetPolicy.rangeIndeterminateReason(
            CameraMotionAnalysisRange(startSeconds: 10, endSeconds: 14)
        ))
        XCTAssertEqual(
            CameraMotionScanBudgetPolicy.rangeIndeterminateReason(
                CameraMotionAnalysisRange(startSeconds: 10, endSeconds: 14.000_001)
            ),
            .analysisBudgetExceeded
        )
        XCTAssertNil(CameraMotionScanBudgetPolicy.sampleIndeterminateReason(
            nativeSampleCount: 1_000
        ))
        XCTAssertEqual(
            CameraMotionScanBudgetPolicy.sampleIndeterminateReason(
                nativeSampleCount: 1_001
            ),
            .analysisBudgetExceeded
        )
    }

    func testReaderTerminalStatusMapsFailuresWithoutMasqueradingAsCoverage() {
        XCTAssertNil(CameraMotionReaderStatusPolicy.indeterminateReason(
            .completed
        ))
        XCTAssertEqual(
            CameraMotionReaderStatusPolicy.indeterminateReason(.failed),
            .videoReadFailed
        )
        for incomplete in [
            CameraMotionReaderTerminalStatus.cancelled,
            .reading,
            .unknown,
        ] {
            XCTAssertEqual(
                CameraMotionReaderStatusPolicy.indeterminateReason(incomplete),
                .incompleteVideoRead
            )
        }
    }

    func testAdapterChecksRangeAndSampleBudgetsBeforeExpensiveVisionWork() throws {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let source = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Offline/CameraMotionVideoAnalyzer.swift"
            ),
            encoding: .utf8
        )
        let rangeBudget = try XCTUnwrap(source.range(
            of: "CameraMotionScanBudgetPolicy.rangeIndeterminateReason(range)"
        ))
        let readiness = try XCTUnwrap(source.range(
            of: "guard isVersionedCalibrationReady"
        ))
        let assetOpen = try XCTUnwrap(source.range(of: "let asset = AVURLAsset(url: url)"))
        XCTAssertLessThan(readiness.lowerBound, rangeBudget.lowerBound)
        XCTAssertLessThan(rangeBudget.lowerBound, assetOpen.lowerBound)

        let sampleBudget = try XCTUnwrap(source.range(
            of: "CameraMotionScanBudgetPolicy.sampleIndeterminateReason("
        ))
        let personVision = try XCTUnwrap(source.range(
            of: "Self.personBoxes(in: buffer)"
        ))
        let retainedByteBudget = try XCTUnwrap(source.range(
            of: "bytesPerRow: CVPixelBufferGetBytesPerRow(buffer)"
        ))
        let peakFeasibility = try XCTUnwrap(source.range(
            of: "CameraBackgroundAnalysisTilePlanner.make("
        ))
        let registrationRequest = try XCTUnwrap(source.range(
            of: "VNTranslationalImageRegistrationRequest("
        ))
        XCTAssertLessThan(sampleBudget.lowerBound, personVision.lowerBound)
        XCTAssertLessThan(sampleBudget.lowerBound, retainedByteBudget.lowerBound)
        XCTAssertLessThan(retainedByteBudget.lowerBound, personVision.lowerBound)
        XCTAssertLessThan(peakFeasibility.lowerBound,
                          registrationRequest.lowerBound)

        let actualRenderSize = try XCTUnwrap(source.range(
            of: "let analysisFrameSize = renderGeometry.pixelDimensions"
        ))
        let calibratedRenderMatch = try XCTUnwrap(source.range(
            of: "analysisFrameSize == calibratedFrameSize"
        ))
        let readerCreation = try XCTUnwrap(source.range(
            of: "let reader = try AVAssetReader(asset: asset)"
        ))
        XCTAssertLessThan(actualRenderSize.lowerBound,
                          calibratedRenderMatch.lowerBound)
        XCTAssertLessThan(calibratedRenderMatch.lowerBound,
                          readerCreation.lowerBound)
    }

    func testUncalibratedPublicAdapterStopsBeforeOpeningVideo() async throws {
        let missing = URL(fileURLWithPath: "/definitely-not-a-camera-fixture.mov")
        let state = try await CameraMotionVideoAnalyzer().analyzeVideo(
            at: missing,
            range: CameraMotionAnalysisRange(startSeconds: 0, endSeconds: 1),
            derivativeWindowSeconds: 0.25
        )
        XCTAssertEqual(state, .calibrationUnavailable)
    }

    func testNativeStreamMustCompleteAndCoverBothRequestedEndpoints() {
        let range = CameraMotionAnalysisRange(startSeconds: 10, endSeconds: 11)

        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: false,
                requestedRange: range,
                firstSampleTimestamp: 10,
                lastSampleTimestamp: 10.98,
                lastSampleDuration: 0.02,
                maximumEndpointGapSeconds: 0.075
            ),
            .incompleteVideoRead
        )
        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 10.10,
                lastSampleTimestamp: 10.98,
                lastSampleDuration: 0.02,
                maximumEndpointGapSeconds: 0.075
            ),
            .incompleteRangeCoverage
        )
        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 9.80,
                lastSampleTimestamp: 11.01,
                lastSampleDuration: 0.02,
                maximumEndpointGapSeconds: 0.075
            ),
            nil,
            "a reader may start early and finish late while fully covering the range"
        )
        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 10,
                lastSampleTimestamp: 10.80,
                lastSampleDuration: 0.02,
                maximumEndpointGapSeconds: 0.075
            ),
            .incompleteRangeCoverage
        )
        XCTAssertNil(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 10.02,
                lastSampleTimestamp: 10.96,
                lastSampleDuration: 0.04,
                maximumEndpointGapSeconds: 0.075
            )
        )
        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 10,
                lastSampleTimestamp: 10.98,
                lastSampleDuration: nil,
                maximumEndpointGapSeconds: 0.075
            ),
            .incompleteRangeCoverage
        )
        XCTAssertEqual(
            CameraMotionStreamCoveragePolicy.indeterminateReason(
                readerCompleted: true,
                requestedRange: range,
                firstSampleTimestamp: 10,
                lastSampleTimestamp: 10.98,
                lastSampleDuration: 0,
                maximumEndpointGapSeconds: 0.075
            ),
            .incompleteRangeCoverage
        )
    }

    func testActualCadenceRejectsDroppedFramesAt240FPS() {
        let nominalStep = 1.0 / 240.0
        for droppedGap in [0.020, 0.050, 0.075] {
            var intervals = Array(repeating: nominalStep, count: 20)
            intervals[10] = droppedGap
            XCTAssertNil(
                cadenceAssessment(
                    intervals: intervals,
                    derivativeWindowSeconds: 8.0 / 240.0
                ),
                "a \(droppedGap * 1_000) ms native gap must not inherit a 75 ms allowance"
            )
        }
    }

    func testReported30FPSCannotLoosenA240FPSDerivativeWindow() {
        // There is deliberately no nominal-rate input to this policy. A stream
        // whose actual PTS are 30 fps cannot satisfy a 240 fps SG window even if
        // container metadata claims a more convenient value.
        XCTAssertNil(cadenceAssessment(
            intervals: Array(repeating: 1.0 / 30.0, count: 12),
            derivativeWindowSeconds: 8.0 / 240.0
        ))
    }

    func testSparsePoseWindowStillAllowsADenser30FPSNativeStream() {
        XCTAssertNotNil(cadenceAssessment(
            intervals: Array(repeating: 1.0 / 30.0, count: 40),
            derivativeWindowSeconds: 16.0
        ))
    }

    func testActualCadenceRejectsADoubled30FPSGap() {
        var intervals = Array(repeating: 1.0 / 30.0, count: 20)
        intervals[10] = 2.0 / 30.0
        XCTAssertNil(cadenceAssessment(
            intervals: intervals,
            derivativeWindowSeconds: 8.0 / 30.0
        ))
    }

    func testActualCadenceAllowsBounded24To30FPSVFR() throws {
        let intervals = (0..<20).map {
            $0.isMultiple(of: 2) ? 1.0 / 24.0 : 1.0 / 30.0
        }
        let assessment = try XCTUnwrap(cadenceAssessment(
            intervals: intervals,
            derivativeWindowSeconds: 8.0 / 24.0
        ))
        XCTAssertEqual(
            assessment.robustNativeFrameIntervalSeconds,
            (1.0 / 24.0 + 1.0 / 30.0) / 2,
            accuracy: 1e-12
        )
        XCTAssertGreaterThanOrEqual(
            assessment.allowedNativeFrameGapSeconds,
            intervals.max() ?? .infinity
        )
    }

    func testActualCadenceRequiresPositiveBoundedSampleDurations() {
        let intervals = Array(repeating: 1.0 / 30.0, count: 12)
        XCTAssertNil(CameraMotionCadencePolicy.assess(
            nativeFrameIntervals: intervals,
            nativeFrameDurations: intervals + [0],
            derivativeWindowSeconds: 8.0 / 30.0,
            hardMaximumGapSeconds: 0.075,
            gapMultiplier: 1.5,
            jitterAllowanceSeconds: 0.005
        ))
        XCTAssertNil(CameraMotionCadencePolicy.assess(
            nativeFrameIntervals: intervals,
            nativeFrameDurations: intervals + [2.0 / 30.0],
            derivativeWindowSeconds: 8.0 / 30.0,
            hardMaximumGapSeconds: 0.075,
            gapMultiplier: 1.5,
            jitterAllowanceSeconds: 0.005
        ))
    }

    func testVideoTrackPolicyRequiresExactlyOneTrack() {
        XCTAssertEqual(
            CameraMotionVideoTrackPolicy.indeterminateReason(videoTrackCount: 0),
            .videoReadFailed
        )
        XCTAssertNil(CameraMotionVideoTrackPolicy.indeterminateReason(videoTrackCount: 1))
        XCTAssertEqual(
            CameraMotionVideoTrackPolicy.indeterminateReason(videoTrackCount: 2),
            .multipleVideoTracks
        )
    }

    func testRenderBudgetPreservesUprightDimensionsAndRejectsOversizeFrames() {
        XCTAssertEqual(
            CameraAnalysisRenderBudget.scaledPixelDimensions(
                uprightSize: CGSize(width: 2_160, height: 3_840),
                maximumPixelCount: 1_920 * 1_080,
                maximumDimensionPixels: 4_096,
                maximumAspectRatio: 4
            ),
            SIMD2(1_080, 1_920)
        )
        XCTAssertEqual(
            CameraAnalysisRenderBudget.acceptedPixelDimensions(
                renderSize: CGSize(width: 1_080, height: 1_920),
                maximumPixelCount: 1_080 * 1_920,
                maximumDimensionPixels: 4_096,
                maximumAspectRatio: 4
            ),
            SIMD2(1_080, 1_920)
        )
        XCTAssertNil(
            CameraAnalysisRenderBudget.acceptedPixelDimensions(
                renderSize: CGSize(width: 2_160, height: 3_840),
                maximumPixelCount: 1_920 * 1_080,
                maximumDimensionPixels: 4_096,
                maximumAspectRatio: 4
            )
        )
    }

    func testRenderAndRetainedBufferBudgetRejectsAxesAspectAndPaddedRows() {
        let maximumPixels = 1_920 * 1_080
        let maximumDimension = 4_096
        let maximumAspectRatio = 4.0
        let paddedBytesPerRow = 8_192
        let exactRetainedBytes = paddedBytesPerRow * 1_080
            * CameraAnalysisBufferBudget.maximumRetainedBufferCount

        XCTAssertTrue(CameraAnalysisBufferBudget.isSupported(
            widthPixels: 1_920,
            heightPixels: 1_080,
            bytesPerRow: paddedBytesPerRow,
            maximumPixelCount: maximumPixels,
            maximumDimensionPixels: maximumDimension,
            maximumAspectRatio: maximumAspectRatio,
            maximumRetainedBytes: exactRetainedBytes
        ))
        XCTAssertFalse(CameraAnalysisBufferBudget.isSupported(
            widthPixels: 1_920,
            heightPixels: 1_080,
            bytesPerRow: paddedBytesPerRow,
            maximumPixelCount: maximumPixels,
            maximumDimensionPixels: maximumDimension,
            maximumAspectRatio: maximumAspectRatio,
            maximumRetainedBytes: exactRetainedBytes - 1
        ))
        XCTAssertFalse(CameraAnalysisBufferBudget.isSupported(
            widthPixels: 4_098,
            heightPixels: 2_048,
            bytesPerRow: 4_098 * 4,
            maximumPixelCount: 10_000_000,
            maximumDimensionPixels: maximumDimension,
            maximumAspectRatio: maximumAspectRatio,
            maximumRetainedBytes: 200_000_000
        ))
        XCTAssertFalse(CameraAnalysisBufferBudget.isSupported(
            widthPixels: 4_000,
            heightPixels: 500,
            bytesPerRow: 4_000 * 4,
            maximumPixelCount: maximumPixels,
            maximumDimensionPixels: maximumDimension,
            maximumAspectRatio: maximumAspectRatio,
            maximumRetainedBytes: 200_000_000
        ))
        XCTAssertFalse(CameraAnalysisBufferBudget.isSupported(
            widthPixels: 2,
            heightPixels: 2,
            bytesPerRow: Int.max,
            maximumPixelCount: maximumPixels,
            maximumDimensionPixels: maximumDimension,
            maximumAspectRatio: maximumAspectRatio,
            maximumRetainedBytes: Int.max
        ))
    }

    func testRenderGeometryContainsIdentityRotatedAndMirroredSourcesWithoutCrop() throws {
        let natural = CGSize(width: 1_920, height: 1_080)
        let identity = try XCTUnwrap(CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: natural,
            preferredTransform: .identity,
            maximumPixelCount: 1_920 * 1_080,
            maximumDimensionPixels: 4_096,
            maximumAspectRatio: 4
        ))
        XCTAssertEqual(identity.pixelDimensions, SIMD2(1_920, 1_080))
        assertSourceBoundsFillOrFitRender(naturalSize: natural, geometry: identity)

        let portraitTransform = CGAffineTransform(
            a: 0, b: 1, c: -1, d: 0, tx: 1_080, ty: 0
        )
        let portrait = try XCTUnwrap(CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: natural,
            preferredTransform: portraitTransform,
            maximumPixelCount: 1_920 * 1_080,
            maximumDimensionPixels: 4_096,
            maximumAspectRatio: 4
        ))
        XCTAssertEqual(portrait.pixelDimensions, SIMD2(1_080, 1_920))
        assertSourceBoundsFillOrFitRender(naturalSize: natural, geometry: portrait)

        let mirroredTransform = CGAffineTransform(
            a: -1, b: 0, c: 0, d: 1, tx: 1_920, ty: 0
        )
        let mirrored = try XCTUnwrap(CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: natural,
            preferredTransform: mirroredTransform,
            maximumPixelCount: 1_920 * 1_080,
            maximumDimensionPixels: 4_096,
            maximumAspectRatio: 4
        ))
        XCTAssertEqual(mirrored.pixelDimensions, SIMD2(1_920, 1_080))
        assertSourceBoundsFillOrFitRender(naturalSize: natural, geometry: mirrored)
    }

    func testVideoFormatGeometryRequiresSquarePixelsFullApertureAndOneRaster() {
        func format(
            _ width: Int = 1_920,
            _ height: Int = 1_080,
            cleanAperture: CGRect? = nil,
            pixelAspectAdjustedDimensions: CGSize? = nil
        ) -> CameraVideoFormatGeometry {
            CameraVideoFormatGeometry(
                encodedDimensions: SIMD2(width, height),
                cleanAperture: cleanAperture
                    ?? CGRect(x: 0, y: 0, width: width, height: height),
                pixelAspectAdjustedDimensions:
                    pixelAspectAdjustedDimensions
                        ?? CGSize(width: width, height: height)
            )
        }
        let natural = CGSize(width: 1_920, height: 1_080)

        XCTAssertTrue(CameraVideoFormatGeometryPolicy.isSupported(
            naturalSize: natural,
            formatGeometries: [format(), format()]
        ))
        XCTAssertFalse(CameraVideoFormatGeometryPolicy.isSupported(
            naturalSize: natural,
            formatGeometries: [format(
                pixelAspectAdjustedDimensions: CGSize(width: 1_440, height: 1_080)
            )]
        ))
        XCTAssertFalse(CameraVideoFormatGeometryPolicy.isSupported(
            naturalSize: natural,
            formatGeometries: [format(
                cleanAperture: CGRect(x: 8, y: 0, width: 1_904, height: 1_080)
            )]
        ))
        XCTAssertFalse(CameraVideoFormatGeometryPolicy.isSupported(
            naturalSize: natural,
            formatGeometries: [format(), format(1_280, 720)]
        ))
        XCTAssertFalse(CameraVideoFormatGeometryPolicy.isSupported(
            naturalSize: natural,
            formatGeometries: []
        ))
    }

    private var configuration: CameraMotionReducer.Configuration {
        reducerConfiguration()
    }

    func testNoMeasurementsFailClosed() {
        XCTAssertEqual(
            CameraMotionReducer.reduce(observations: [], configuration: configuration),
            .indeterminate(.insufficientMeasurements)
        )
    }

    func testSessionOnlyAndTemporalPermissionsAreDistinct() {
        XCTAssertFalse(CameraReferenceState.unmeasured.permitsStaticEquilibrium)
        XCTAssertFalse(CameraReferenceState.unmeasured.permitsTemporalDynamics)
        XCTAssertTrue(CameraReferenceState.notRequiredForSingleFrame.permitsStaticEquilibrium)
        XCTAssertFalse(CameraReferenceState.notRequiredForSingleFrame.permitsTemporalDynamics)

        let evidence = CameraMotionEvidence(
            analyzedDurationSeconds: 1,
            derivativeWindowSeconds: 0.25,
            peakNormalizedTranslation: 0,
            translationStaticUpperBound: 0.01,
            translationMovingLowerBound: 0.02,
            peakRotationRadians: 0,
            rotationStaticUpperBoundRadians: 0.02,
            rotationMovingLowerBoundRadians: 0.04,
            peakScaleFraction: 0,
            scaleStaticUpperBound: 0.01,
            scaleMovingLowerBound: 0.02,
            calibrationProfileID: "synthetic-unit-test-v1",
            analysisFrameWidthPixels: 1_280,
            analysisFrameHeightPixels: 720,
            maximumAnalysisPixelCount: 1_920 * 1_080,
            measuredIntervals: 30,
            sampledFrames: 31
        )
        XCTAssertTrue(CameraReferenceState.staticWithinBudget(evidence)
            .permitsStaticEquilibrium)
        XCTAssertTrue(CameraReferenceState.staticWithinBudget(evidence)
            .permitsTemporalDynamics)
        XCTAssertFalse(CameraReferenceState.moving(evidence).permitsStaticEquilibrium)
        XCTAssertFalse(CameraReferenceState.moving(evidence).permitsTemporalDynamics)
        XCTAssertFalse(CameraReferenceState.indeterminate(.insufficientBackground)
            .permitsStaticEquilibrium)
        XCTAssertFalse(CameraReferenceState.indeterminate(.insufficientBackground)
            .permitsTemporalDynamics)
    }

    func testTranslationUsesStaticAmbiguousAndMovingBands() throws {
        let staticBound = configuration.staticMaximumNormalizedTranslation
        let movingBound = configuration.movingMinimumNormalizedTranslation
        let atBudget = observations(windowTranslations: [
            SIMD2(0.002, 0), SIMD2(0.004, 0), SIMD2(0.006, 0),
            SIMD2(0.008, 0), SIMD2(staticBound, 0),
        ])
        let ambiguous = observations(windowTranslations: [
            SIMD2(0.002, 0), SIMD2(0.004, 0), SIMD2(0.006, 0),
            SIMD2(0.008, 0), SIMD2(staticBound + 0.0001, 0),
        ])
        let moving = observations(windowTranslations: [
            SIMD2(0.002, 0), SIMD2(0.004, 0), SIMD2(0.006, 0),
            SIMD2(0.008, 0), SIMD2(movingBound, 0),
        ])

        guard case .staticWithinBudget(let staticEvidence) = CameraMotionReducer.reduce(
            observations: atBudget,
            configuration: configuration
        ) else {
            return XCTFail("translation equal to the registered budget must remain admissible")
        }
        XCTAssertEqual(staticEvidence.peakNormalizedTranslation, staticBound, accuracy: 1e-12)
        XCTAssertEqual(staticEvidence.translationStaticUpperBound, staticBound, accuracy: 1e-12)

        guard case .betweenCalibrationBands(let ambiguousEvidence) = CameraMotionReducer.reduce(
            observations: ambiguous,
            configuration: configuration
        ) else {
            return XCTFail("measurements between the registered bands must fail closed")
        }
        XCTAssertGreaterThan(ambiguousEvidence.peakNormalizedTranslation, staticBound)
        XCTAssertLessThan(ambiguousEvidence.peakNormalizedTranslation, movingBound)

        guard case .moving(let movingEvidence) = CameraMotionReducer.reduce(
            observations: moving,
            configuration: configuration
        ) else {
            return XCTFail("translation at the registered moving bound must be moving")
        }
        XCTAssertEqual(movingEvidence.peakNormalizedTranslation, movingBound, accuracy: 1e-12)
    }

    func testStaticBandHasNoPermissionSideFloatingPointTolerance() {
        let justAboveStatic = configuration.staticMaximumScaleFraction.nextUp
        let samples = observations(
            windowTranslations: Array(repeating: .zero, count: 5),
            scales: [0, 0, 0, 0, justAboveStatic]
        )

        guard case .betweenCalibrationBands(let evidence) =
                CameraMotionReducer.reduce(
                    observations: samples,
                    configuration: configuration
                ) else {
            return XCTFail("any representable value above the static band must fail closed")
        }
        XCTAssertEqual(evidence.peakScaleFraction, justAboveStatic)
    }

    func testWindowAnchorCatchesOutAndBackMotionWhoseEndpointsAliasToZero() {
        let samples = observations(windowTranslations: [
            SIMD2(0.03, 0),
            SIMD2(0.06, 0),
            SIMD2(0.03, 0),
            SIMD2(0, 0),
            SIMD2(0, 0),
        ])

        guard case .moving(let evidence) = CameraMotionReducer.reduce(
            observations: samples,
            configuration: configuration
        ) else {
            return XCTFail("the midpoint excursion must survive a zero net endpoint displacement")
        }
        XCTAssertEqual(samples.last?.normalizedWindowTranslation, .zero)
        XCTAssertEqual(evidence.peakNormalizedTranslation, 0.06, accuracy: 1e-12)
    }

    func testHighFrameRateRegistrationNoiseIsNotSummedIntoMotion() {
        let frameInterval = 1.0 / 240.0
        let samples = (0..<60).map { index in
            observation(
                from: Double(index) * frameInterval,
                to: Double(index + 1) * frameInterval,
                baseline: 0,
                stepTranslation: SIMD2(index.isMultiple(of: 2) ? 0.001 : -0.001, 0),
                windowTranslation: SIMD2(index.isMultiple(of: 2) ? 0.001 : 0, 0)
            )
        }

        let highRateConfiguration = reducerConfiguration(
            robustNativeFrameIntervalSeconds: frameInterval,
            jitterAllowanceSeconds: 0.005,
            nativeFrameIntervalDomainSeconds: frameInterval...frameInterval
        )
        guard case .staticWithinBudget(let evidence) = CameraMotionReducer.reduce(
            observations: samples,
            configuration: highRateConfiguration
        ) else {
            return XCTFail("bounded registration noise must not grow linearly with frame rate")
        }
        XCTAssertEqual(evidence.peakNormalizedTranslation, 0.001, accuracy: 1e-12)
    }

    func testRotationAndScaleHaveIndependentBands() {
        let atBudget = observations(
            windowTranslations: Array(repeating: .zero, count: 5),
            rotations: [0.004, 0.008, 0.012, 0.016, 0.02]
        )
        let rotating = observations(
            windowTranslations: Array(repeating: .zero, count: 5),
            rotations: [0.004, 0.008, 0.012, 0.016, 0.04]
        )
        let zooming = observations(
            windowTranslations: Array(repeating: .zero, count: 5),
            scales: [0.002, 0.004, 0.008, 0.012, 0.02]
        )

        guard case .staticWithinBudget(let evidence) = CameraMotionReducer.reduce(
            observations: atBudget,
            configuration: configuration
        ) else {
            return XCTFail("rotation equal to the registered budget must remain admissible")
        }
        XCTAssertEqual(evidence.peakRotationRadians, 0.02, accuracy: 1e-12)

        guard case .moving = CameraMotionReducer.reduce(
            observations: rotating,
            configuration: configuration
        ) else {
            return XCTFail("rotation at the moving bound must withhold dynamics")
        }
        guard case .moving(let scaleEvidence) = CameraMotionReducer.reduce(
            observations: zooming,
            configuration: configuration
        ) else {
            return XCTFail("zoom at the moving bound must withhold dynamics")
        }
        XCTAssertEqual(scaleEvidence.peakScaleFraction, 0.02, accuracy: 1e-12)
    }

    func testOneFrameRotationAndScalePeaksCannotHideAtAnchorBoundaries() {
        var rotating = observations(
            windowTranslations: Array(repeating: .zero, count: 5)
        )
        rotating[2] = observation(
            from: 0.10,
            to: 0.15,
            baseline: 0,
            windowTranslation: .zero,
            stepRotation: configuration.movingMinimumRotationRadians
        )
        guard case .moving(let rotationEvidence) = CameraMotionReducer.reduce(
            observations: rotating,
            configuration: configuration
        ) else {
            return XCTFail("a native-step rotation must survive anchor replacement")
        }
        XCTAssertEqual(
            rotationEvidence.peakRotationRadians,
            configuration.movingMinimumRotationRadians,
            accuracy: 1e-12
        )

        var scaling = observations(
            windowTranslations: Array(repeating: .zero, count: 5)
        )
        scaling[2] = observation(
            from: 0.10,
            to: 0.15,
            baseline: 0,
            windowTranslation: .zero,
            stepScale: configuration.movingMinimumScaleFraction
        )
        guard case .moving(let scaleEvidence) = CameraMotionReducer.reduce(
            observations: scaling,
            configuration: configuration
        ) else {
            return XCTFail("a native-step scale change must survive anchor replacement")
        }
        XCTAssertEqual(
            scaleEvidence.peakScaleFraction,
            configuration.movingMinimumScaleFraction,
            accuracy: 1e-12
        )
    }

    func testWithinAllowedGapOutAndBackMotionCannotBecomeStatic() {
        var samples = observations(
            windowTranslations: Array(repeating: .zero, count: 5)
        )
        samples[2] = observation(
            from: 0.10,
            to: 0.15,
            baseline: 0,
            stepTranslation: SIMD2(0.02, 0),
            windowTranslation: .zero
        )
        guard case .moving(let evidence) = CameraMotionReducer.reduce(
            observations: samples,
            configuration: configuration
        ) else {
            return XCTFail("an out-and-back excursion inside the cadence gap must be moving")
        }
        XCTAssertEqual(evidence.peakNormalizedTranslation, 0.02, accuracy: 1e-12)
    }

    func testMissingCalibrationProfileCannotClaimStaticOrMoving() {
        let uncalibrated = reducerConfiguration(profileID: nil)
        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        guard case .calibrationRequired(let evidence) = CameraMotionReducer.reduce(
            observations: measurements,
            configuration: uncalibrated
        ) else {
            return XCTFail("an implementation profile is not a validated camera calibration")
        }
        XCTAssertNil(evidence.calibrationProfileID)
    }

    func testCalibrationProfileMustMatchActualAnalysisDimensions() {
        let mismatched = reducerConfiguration(
            analysisFrameSizePixels: SIMD2(1_080, 1_920),
            calibrationFrameSizePixels: SIMD2(1_280, 720)
        )
        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )

        guard case .calibrationRequired(let evidence) = CameraMotionReducer.reduce(
            observations: measurements,
            configuration: mismatched
        ) else {
            return XCTFail("a calibration profile cannot move between render dimensions")
        }
        XCTAssertEqual(evidence.analysisFrameWidthPixels, 1_080)
        XCTAssertEqual(evidence.analysisFrameHeightPixels, 1_920)
    }

    func testReducerRejectsUnsupportedDirectAnalysisFrameGeometry() {
        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        let oddFrame = reducerConfiguration(
            analysisFrameSizePixels: SIMD2(1_279, 719),
            calibrationFrameSizePixels: SIMD2(1_279, 719)
        )

        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: measurements,
                configuration: oddFrame
            ),
            .indeterminate(.invalidMeasurement)
        )
    }

    func testReducerRequiresTypedProfileAndBoundedRuntimeWindowCadenceDomain() {
        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        for refused in [
            reducerConfiguration(profileID: ""),
            reducerConfiguration(
                derivativeWindowDomainSeconds: 0.30...0.40
            ),
            reducerConfiguration(
                nativeFrameIntervalDomainSeconds: 0.01...0.02
            ),
            reducerConfiguration(
                derivativeWindowDomainSeconds:
                    (0.25 + 2e-12)...(0.25 + 2e-12)
            ),
            reducerConfiguration(
                nativeFrameIntervalDomainSeconds:
                    (0.05 + 2e-12)...(0.05 + 2e-12)
            ),
            reducerConfiguration(fingerprintStaticTranslation: 0.009),
        ] {
            guard case .calibrationRequired = CameraMotionReducer.reduce(
                observations: measurements,
                configuration: refused
            ) else {
                return XCTFail(
                    "blank, drifted or out-of-domain profile must not reuse static bands"
                )
            }
        }

        let representationNoise = reducerConfiguration(
            derivativeWindowDomainSeconds:
                (0.25 + 0.5e-12)...(0.25 + 0.5e-12),
            nativeFrameIntervalDomainSeconds:
                (0.05 + 0.5e-12)...(0.05 + 0.5e-12)
        )
        guard case .staticWithinBudget = CameraMotionReducer.reduce(
            observations: measurements,
            configuration: representationNoise
        ) else {
            return XCTFail(
                "sub-tolerance time representation noise must remain calibrated"
            )
        }
    }

    func testReducerSecondBoundaryRejectsEveryResourceAndPeakFingerprintDrift() {
        let measurements = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        let refused = [
            ("maximum analysis dimension", reducerConfiguration(
                fingerprintMaximumAnalysisDimensionPixels: 4_094
            )),
            ("maximum analysis aspect ratio", reducerConfiguration(
                fingerprintMaximumAnalysisAspectRatio: 3.5
            )),
            ("retained byte budget", reducerConfiguration(
                fingerprintMaximumRetainedPixelBufferBytes:
                    63 * 1_024 * 1_024
            )),
            ("retained buffer count", reducerConfiguration(
                fingerprintMaximumRetainedPixelBufferCount:
                    CameraAnalysisBufferBudget.maximumRetainedBufferCount - 1
            )),
            ("box average size", reducerConfiguration(
                fingerprintAppearanceBoxAverageSizePixels: 4
            )),
            ("box average spacing", reducerConfiguration(
                fingerprintAppearanceBoxAverageSpacingPixels: 16
            )),
            ("correlation sample spacing", reducerConfiguration(
                fingerprintRegistrationCorrelationSampleSpacingPixels: 24
            )),
            ("peak search radius", reducerConfiguration(
                fingerprintRegistrationPeakSearchRadiusPixels: 40
            )),
            ("peak separation", reducerConfiguration(
                fingerprintRegistrationPeakMinimumSeparationPixels: 24
            )),
            ("minimum peak correlation", reducerConfiguration(
                fingerprintMinimumRegistrationPeakCorrelation: 0.55
            )),
            ("correlation pair cap", reducerConfiguration(
                fingerprintMaximumRegistrationCorrelationPairCountPerTile:
                    CameraRegistrationPeakAnalyzer
                        .maximumCorrelationPairCountPerTile - 1
            )),
        ]

        for (field, configuration) in refused {
            guard case .calibrationRequired = CameraMotionReducer.reduce(
                observations: measurements,
                configuration: configuration
            ) else {
                XCTFail(
                    "the reducer must independently bind \(field)"
                )
                continue
            }
        }

        let malformed = [
            ("retained buffer count", reducerConfiguration(
                fingerprintMaximumRetainedPixelBufferCount: 0
            )),
            ("minimum alias overlap", reducerConfiguration(
                fingerprintRegistrationAliasMinimumOverlapPairCount: 0
            )),
            ("shared alias domain", reducerConfiguration(
                fingerprintRegistrationAliasSharedDomainSideSamples: 0
            )),
            ("tail alias pairs", reducerConfiguration(
                fingerprintRegistrationAliasTailPairCount: 0
            )),
            ("correlation pair cap", reducerConfiguration(
                fingerprintMaximumRegistrationCorrelationPairCountPerTile: 0
            )),
        ]
        for (field, configuration) in malformed {
            XCTAssertEqual(
                CameraMotionReducer.reduce(
                    observations: measurements,
                    configuration: configuration
                ),
                .indeterminate(.invalidMeasurement),
                "non-positive \(field) must remain structurally invalid"
            )
        }
    }

    func testReducerRecomputesAllowedGapFromActualObservations() {
        var intervals = Array(repeating: 1.0 / 30.0, count: 20)
        intervals[10] = 2.0 / 30.0
        var timestamp = 0.0
        let measurements = intervals.map { interval in
            defer { timestamp += interval }
            return observation(
                from: timestamp,
                to: timestamp + interval,
                baseline: max(0, timestamp - 0.20),
                windowTranslation: SIMD2(0.001, 0)
            )
        }
        let window = 8.0 / 30.0
        let typed = reducerConfiguration(
            derivativeWindowSeconds: window,
            robustNativeFrameIntervalSeconds: 1.0 / 30.0,
            jitterAllowanceSeconds: 0.005,
            derivativeWindowDomainSeconds: window...window,
            nativeFrameIntervalDomainSeconds: (1.0 / 30.0)...(1.0 / 30.0)
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: measurements,
                configuration: typed
            ),
            .indeterminate(.timestampDiscontinuity)
        )
    }

    func testLowQualityAndInsufficientBackgroundFailClosed() {
        var lowQuality = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        lowQuality[2] = observation(
            from: 0.10,
            to: 0.15,
            baseline: 0,
            windowTranslation: SIMD2(0.001, 0),
            registrationQuality: 0.49
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: lowQuality,
                configuration: configuration
            ),
            .indeterminate(.lowRegistrationQuality)
        )

        var noBackground = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        noBackground[1] = observation(
            from: 0.05,
            to: 0.10,
            baseline: 0,
            windowTranslation: SIMD2(0.001, 0),
            backgroundFraction: 0.19
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: noBackground,
                configuration: configuration
            ),
            .indeterminate(.insufficientBackground)
        )
    }

    func testPTSDiscontinuityAndShortCoverageFailClosed() {
        var discontinuous = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 5)
        )
        discontinuous[3] = observation(
            from: 0.20,
            to: 0.25,
            baseline: 0.20,
            windowTranslation: SIMD2(0.001, 0)
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: discontinuous,
                configuration: configuration
            ),
            .indeterminate(.timestampDiscontinuity)
        )

        let tooShort = observations(
            windowTranslations: Array(repeating: SIMD2(0.001, 0), count: 4)
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(
                observations: tooShort,
                configuration: configuration
            ),
            .indeterminate(.insufficientCoverage)
        )
    }

    func testVariableFrameRateUsesActualPTSForCoverage() {
        let samples = [
            observation(from: 0.00, to: 0.04, baseline: 0.00,
                        windowTranslation: SIMD2(0.002, 0)),
            observation(from: 0.04, to: 0.10, baseline: 0.00,
                        windowTranslation: SIMD2(0.004, 0)),
            observation(from: 0.10, to: 0.15, baseline: 0.00,
                        windowTranslation: SIMD2(0.006, 0)),
            observation(from: 0.15, to: 0.21, baseline: 0.00,
                        windowTranslation: SIMD2(0.008, 0)),
            observation(from: 0.21, to: 0.25, baseline: 0.00,
                        windowTranslation: SIMD2(0.010, 0)),
        ]

        guard case .staticWithinBudget(let evidence) = CameraMotionReducer.reduce(
            observations: samples,
            configuration: configuration
        ) else {
            return XCTFail("the exact 250 ms VFR coverage must be admitted")
        }
        XCTAssertEqual(evidence.analyzedDurationSeconds, 0.25, accuracy: 1e-12)
        XCTAssertEqual(evidence.peakNormalizedTranslation, 0.01, accuracy: 1e-12)
        XCTAssertEqual(evidence.measuredIntervals, 5)
    }

    func testMaximumGapDoesNotInvalidate120Or240FPSDerivativeWindows() {
        for rate in [120.0, 240.0] {
            let window = Double(SavitzkyGolayFilter.windowSize - 1) / rate
            let interval = 1 / rate
            let highRateConfiguration = reducerConfiguration(
                derivativeWindowSeconds: window,
                robustNativeFrameIntervalSeconds: interval,
                jitterAllowanceSeconds: 0.005,
                derivativeWindowDomainSeconds: window...window,
                nativeFrameIntervalDomainSeconds: interval...interval,
                profileID: "synthetic-high-rate-v1"
            )
            let samples = (0...SavitzkyGolayFilter.windowSize).map { index in
                observation(
                    from: Double(index) * interval,
                    to: Double(index + 1) * interval,
                    baseline: Double(index) * interval,
                    windowTranslation: SIMD2(0.001, 0)
                )
            }

            guard case .staticWithinBudget = CameraMotionReducer.reduce(
                observations: samples,
                configuration: highRateConfiguration
            ) else {
                return XCTFail("\(Int(rate)) fps must not be rejected by configuration validation")
            }
        }
    }

    func testInvalidNumbersAndImpossibleWindowBaselinesFailClosed() {
        var invalid = observations(
            windowTranslations: Array(repeating: .zero, count: 5)
        )
        invalid[0] = observation(
            from: 0,
            to: 0.05,
            baseline: 0,
            windowTranslation: SIMD2(Double.nan, 0)
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(observations: invalid, configuration: configuration),
            .indeterminate(.invalidMeasurement)
        )

        var futureBaseline = observations(
            windowTranslations: Array(repeating: .zero, count: 5)
        )
        futureBaseline[2] = observation(
            from: 0.10,
            to: 0.15,
            baseline: 0.11,
            windowTranslation: .zero
        )
        XCTAssertEqual(
            CameraMotionReducer.reduce(observations: futureBaseline,
                                       configuration: configuration),
            .indeterminate(.invalidMeasurement)
        )
    }

    private func observations(
        windowTranslations: [SIMD2<Double>],
        rotations: [Double]? = nil,
        scales: [Double]? = nil
    ) -> [CameraMotionObservation] {
        precondition(rotations == nil || rotations?.count == windowTranslations.count)
        precondition(scales == nil || scales?.count == windowTranslations.count)
        return windowTranslations.enumerated().map { index, translation in
            observation(
                from: Double(index) * 0.05,
                to: Double(index + 1) * 0.05,
                baseline: 0,
                windowTranslation: translation,
                rotation: rotations?[index] ?? 0,
                scale: scales?[index] ?? 0
            )
        }
    }

    private func observation(
        from: TimeInterval,
        to: TimeInterval,
        baseline: TimeInterval,
        stepTranslation: SIMD2<Double> = .zero,
        windowTranslation: SIMD2<Double>,
        stepRotation: Double = 0,
        rotation: Double = 0,
        stepScale: Double = 0,
        scale: Double = 0,
        registrationQuality: Double = 0.9,
        backgroundFraction: Double = 0.8
    ) -> CameraMotionObservation {
        CameraMotionObservation(
            startTimestamp: from,
            endTimestamp: to,
            translationWindowBaselineTimestamp: baseline,
            rotationWindowBaselineTimestamp: baseline,
            scaleWindowBaselineTimestamp: baseline,
            normalizedStepTranslation: stepTranslation,
            normalizedWindowTranslation: windowTranslation,
            stepRotationRadians: stepRotation,
            windowRotationRadians: rotation,
            stepScaleFraction: stepScale,
            windowScaleFraction: scale,
            registrationQuality: registrationQuality,
            backgroundFraction: backgroundFraction
        )
    }

    private func reducerConfiguration(
        derivativeWindowSeconds: TimeInterval = 0.25,
        robustNativeFrameIntervalSeconds: TimeInterval = 0.05,
        hardMaximumGapSeconds: TimeInterval = 0.075,
        gapMultiplier: Double = 1.5,
        jitterAllowanceSeconds: TimeInterval = 0.02,
        analysisFrameSizePixels: SIMD2<Int> = SIMD2(1_280, 720),
        calibrationFrameSizePixels: SIMD2<Int> = SIMD2(1_280, 720),
        derivativeWindowDomainSeconds: ClosedRange<TimeInterval> = 0.20...0.30,
        nativeFrameIntervalDomainSeconds: ClosedRange<TimeInterval> = 0.04...0.06,
        profileID: String? = "synthetic-unit-test-v1",
        fingerprintStaticTranslation: Double? = nil,
        maximumAnalysisDimensionPixels: Int = 4_096,
        fingerprintMaximumAnalysisDimensionPixels: Int? = nil,
        maximumAnalysisAspectRatio: Double = 4,
        fingerprintMaximumAnalysisAspectRatio: Double? = nil,
        maximumRetainedPixelBufferBytes: Int = 64 * 1_024 * 1_024,
        fingerprintMaximumRetainedPixelBufferBytes: Int? = nil,
        fingerprintMaximumRetainedPixelBufferCount: Int? = nil,
        appearanceBoxAverageSizePixels: Int = 8,
        fingerprintAppearanceBoxAverageSizePixels: Int? = nil,
        appearanceBoxAverageSpacingPixels: Int = 8,
        fingerprintAppearanceBoxAverageSpacingPixels: Int? = nil,
        registrationCorrelationSampleSpacingPixels: Int = 32,
        fingerprintRegistrationCorrelationSampleSpacingPixels: Int? = nil,
        registrationPeakSearchRadiusPixels: Int = 48,
        fingerprintRegistrationPeakSearchRadiusPixels: Int? = nil,
        registrationPeakMinimumSeparationPixels: Int = 16,
        fingerprintRegistrationPeakMinimumSeparationPixels: Int? = nil,
        minimumRegistrationPeakCorrelation: Double = 0.5,
        fingerprintMinimumRegistrationPeakCorrelation: Double? = nil,
        fingerprintMaximumRegistrationCorrelationPairCountPerTile: Int? = nil,
        fingerprintRegistrationAliasMinimumOverlapPairCount: Int? = nil,
        fingerprintRegistrationAliasSharedDomainSideSamples: Int? = nil,
        fingerprintRegistrationAliasTailPairCount: Int? = nil,
        fingerprintHumanRectanglesRequestRevision: Int? = nil,
        fingerprintTranslationalRegistrationRequestRevision: Int? = nil
    ) -> CameraMotionReducer.Configuration {
        let allowedGap = min(
            hardMaximumGapSeconds,
            robustNativeFrameIntervalSeconds * gapMultiplier
                + jitterAllowanceSeconds
        )
        let fingerprint = profileID.map {
            CameraMotionCalibrationFingerprint(
                profileID: $0,
                analysisFrameSizePixels: calibrationFrameSizePixels,
                adapterRevision:
                    CameraMotionVideoAnalyzer.calibrationAdapterRevision,
                humanRectanglesRequestRevision:
                    fingerprintHumanRectanglesRequestRevision
                        ?? CameraMotionVideoAnalyzer
                            .humanRectanglesRequestRevision,
                translationalRegistrationRequestRevision:
                    fingerprintTranslationalRegistrationRequestRevision
                        ?? CameraMotionVideoAnalyzer
                            .translationalRegistrationRequestRevision,
                provisionalStaticTranslation:
                    fingerprintStaticTranslation ?? 0.01,
                provisionalMovingTranslation: 0.02,
                provisionalStaticRotationRadians: 0.02,
                provisionalMovingRotationRadians: 0.04,
                provisionalStaticScaleFraction: 0.01,
                provisionalMovingScaleFraction: 0.02,
                maximumNativeFrameGapSeconds: hardMaximumGapSeconds,
                nativeFrameGapMultiplier: gapMultiplier,
                nativeFrameJitterAllowanceSeconds: jitterAllowanceSeconds,
                maximumAnalysisPixelCount: 1_920 * 1_080,
                maximumAnalysisDimensionPixels:
                    fingerprintMaximumAnalysisDimensionPixels
                        ?? maximumAnalysisDimensionPixels,
                maximumAnalysisAspectRatio:
                    fingerprintMaximumAnalysisAspectRatio
                        ?? maximumAnalysisAspectRatio,
                maximumRetainedPixelBufferBytes:
                    fingerprintMaximumRetainedPixelBufferBytes
                        ?? maximumRetainedPixelBufferBytes,
                maximumRetainedPixelBufferCount:
                    fingerprintMaximumRetainedPixelBufferCount
                        ?? CameraAnalysisBufferBudget.maximumRetainedBufferCount,
                appearanceBoxAverageSizePixels:
                    fingerprintAppearanceBoxAverageSizePixels
                        ?? appearanceBoxAverageSizePixels,
                appearanceBoxAverageSpacingPixels:
                    fingerprintAppearanceBoxAverageSpacingPixels
                        ?? appearanceBoxAverageSpacingPixels,
                registrationCorrelationSampleSpacingPixels:
                    fingerprintRegistrationCorrelationSampleSpacingPixels
                        ?? registrationCorrelationSampleSpacingPixels,
                registrationPeakSearchRadiusPixels:
                    fingerprintRegistrationPeakSearchRadiusPixels
                        ?? registrationPeakSearchRadiusPixels,
                registrationPeakMinimumSeparationPixels:
                    fingerprintRegistrationPeakMinimumSeparationPixels
                        ?? registrationPeakMinimumSeparationPixels,
                registrationAliasMinimumOverlapPairCount:
                    fingerprintRegistrationAliasMinimumOverlapPairCount
                        ?? CameraRegistrationPeakAnalyzer
                            .aliasMinimumOverlapPairCount,
                registrationAliasSharedDomainSideSamples:
                    fingerprintRegistrationAliasSharedDomainSideSamples
                        ?? CameraRegistrationPeakAnalyzer
                            .aliasSharedDomainSideSamples,
                registrationAliasTailPairCount:
                    fingerprintRegistrationAliasTailPairCount
                        ?? CameraRegistrationPeakAnalyzer.aliasTailPairCount,
                maximumRegistrationCorrelationPairCountPerTile:
                    fingerprintMaximumRegistrationCorrelationPairCountPerTile
                        ?? CameraRegistrationPeakAnalyzer
                            .maximumCorrelationPairCountPerTile,
                maximumScanDurationSeconds:
                    CameraMotionScanBudgetPolicy.maximumRangeDurationSeconds,
                maximumNativeSampleCount:
                    CameraMotionScanBudgetPolicy.maximumNativeSampleCount,
                personBoxInflationFraction: 0.03,
                minimumTileAreaFraction: 0.01,
                minimumBackgroundFraction: 0.2,
                minimumRegistrationQuality: 0.5,
                minimumRegistrationPeakCorrelation:
                    fingerprintMinimumRegistrationPeakCorrelation
                        ?? minimumRegistrationPeakCorrelation,
                minimumTileCount: 3,
                minimumStructureTensorEigenvalue: 0.0005,
                minimumRegistrationUniqueness: 0.05,
                maximumPostWarpMeanAbsoluteError: 20,
                maximumFitResidualPixels: 2,
                minimumSpatialEigenvalueFraction: 0.001,
                derivativeWindowDomainSeconds:
                    derivativeWindowDomainSeconds,
                nativeFrameIntervalDomainSeconds:
                    nativeFrameIntervalDomainSeconds
            )
        }
        return CameraMotionReducer.Configuration(
            derivativeWindowSeconds: derivativeWindowSeconds,
            allowedNativeFrameGapSeconds: allowedGap,
            robustNativeFrameIntervalSeconds:
                robustNativeFrameIntervalSeconds,
            staticMaximumNormalizedTranslation: 0.01,
            movingMinimumNormalizedTranslation: 0.02,
            staticMaximumRotationRadians: 0.02,
            movingMinimumRotationRadians: 0.04,
            staticMaximumScaleFraction: 0.01,
            movingMinimumScaleFraction: 0.02,
            minimumRegistrationQuality: 0.5,
            minimumRegistrationPeakCorrelation:
                minimumRegistrationPeakCorrelation,
            minimumBackgroundFraction: 0.2,
            analysisFrameSizePixels: analysisFrameSizePixels,
            maximumAnalysisPixelCount: 1_920 * 1_080,
            maximumAnalysisDimensionPixels: maximumAnalysisDimensionPixels,
            maximumAnalysisAspectRatio: maximumAnalysisAspectRatio,
            maximumRetainedPixelBufferBytes:
                maximumRetainedPixelBufferBytes,
            appearanceBoxAverageSizePixels: appearanceBoxAverageSizePixels,
            appearanceBoxAverageSpacingPixels:
                appearanceBoxAverageSpacingPixels,
            registrationCorrelationSampleSpacingPixels:
                registrationCorrelationSampleSpacingPixels,
            registrationPeakSearchRadiusPixels:
                registrationPeakSearchRadiusPixels,
            registrationPeakMinimumSeparationPixels:
                registrationPeakMinimumSeparationPixels,
            calibrationFingerprint: fingerprint
        )
    }

    private func videoConfiguration(
        profileID: String?,
        frameSizePixels: SIMD2<Int>?,
        includesFingerprint: Bool = true,
        fingerprintProfileID: String? = nil,
        fingerprintFrameSizePixels: SIMD2<Int>? = nil,
        fingerprintAdapterRevision: Int =
            CameraMotionVideoAnalyzer.calibrationAdapterRevision,
        fingerprintHumanRectanglesRequestRevision: Int? = nil,
        fingerprintTranslationalRegistrationRequestRevision: Int? = nil,
        provisionalStaticTranslation: Double = 0.01,
        fingerprintStaticTranslation: Double? = nil,
        maximumNativeFrameGapSeconds: Double = 0.075,
        fingerprintMaximumNativeFrameGapSeconds: Double? = nil,
        nativeFrameGapMultiplier: Double = 1.5,
        nativeFrameJitterAllowanceSeconds: TimeInterval = 0.005,
        maximumAnalysisPixelCount: Int = 1_920 * 1_080,
        fingerprintMaximumAnalysisPixelCount: Int? = nil,
        maximumAnalysisDimensionPixels: Int = 4_096,
        fingerprintMaximumAnalysisDimensionPixels: Int? = nil,
        maximumAnalysisAspectRatio: Double = 4,
        fingerprintMaximumAnalysisAspectRatio: Double? = nil,
        maximumRetainedPixelBufferBytes: Int = 64 * 1_024 * 1_024,
        fingerprintMaximumRetainedPixelBufferBytes: Int? = nil,
        fingerprintMaximumRetainedPixelBufferCount: Int? = nil,
        appearanceBoxAverageSizePixels: Int = 8,
        fingerprintAppearanceBoxAverageSizePixels: Int? = nil,
        appearanceBoxAverageSpacingPixels: Int = 8,
        fingerprintAppearanceBoxAverageSpacingPixels: Int? = nil,
        registrationCorrelationSampleSpacingPixels: Int = 32,
        fingerprintRegistrationCorrelationSampleSpacingPixels: Int? = nil,
        registrationPeakSearchRadiusPixels: Int = 48,
        fingerprintRegistrationPeakSearchRadiusPixels: Int? = nil,
        registrationPeakMinimumSeparationPixels: Int = 16,
        fingerprintRegistrationPeakMinimumSeparationPixels: Int? = nil,
        fingerprintRegistrationAliasMinimumOverlapPairCount: Int? = nil,
        fingerprintRegistrationAliasSharedDomainSideSamples: Int? = nil,
        fingerprintRegistrationAliasTailPairCount: Int? = nil,
        fingerprintMaximumRegistrationCorrelationPairCountPerTile: Int? = nil,
        personBoxInflationFraction: CGFloat = 0.03,
        fingerprintPersonBoxInflationFraction: CGFloat? = nil,
        minimumRegistrationQuality: Double = 0.5,
        fingerprintMinimumRegistrationQuality: Double? = nil,
        minimumRegistrationPeakCorrelation: Double = 0.5,
        fingerprintMinimumRegistrationPeakCorrelation: Double? = nil,
        minimumRegistrationUniqueness: Double = 0.05,
        fingerprintMinimumRegistrationUniqueness: Double? = nil,
        derivativeWindowDomainSeconds: ClosedRange<TimeInterval> = 0.20...0.30,
        nativeFrameIntervalDomainSeconds: ClosedRange<TimeInterval> = 0.03...0.06
    ) -> CameraMotionVideoAnalyzer.Configuration {
        let fingerprint: CameraMotionCalibrationFingerprint?
        if includesFingerprint, let profileID, let frameSizePixels {
            fingerprint = CameraMotionCalibrationFingerprint(
                profileID: fingerprintProfileID ?? profileID,
                analysisFrameSizePixels:
                    fingerprintFrameSizePixels ?? frameSizePixels,
                adapterRevision: fingerprintAdapterRevision,
                humanRectanglesRequestRevision:
                    fingerprintHumanRectanglesRequestRevision
                        ?? CameraMotionVideoAnalyzer
                            .humanRectanglesRequestRevision,
                translationalRegistrationRequestRevision:
                    fingerprintTranslationalRegistrationRequestRevision
                        ?? CameraMotionVideoAnalyzer
                            .translationalRegistrationRequestRevision,
                provisionalStaticTranslation:
                    fingerprintStaticTranslation ?? provisionalStaticTranslation,
                provisionalMovingTranslation: 0.02,
                provisionalStaticRotationRadians: 0.01,
                provisionalMovingRotationRadians: 0.02,
                provisionalStaticScaleFraction: 0.01,
                provisionalMovingScaleFraction: 0.02,
                maximumNativeFrameGapSeconds:
                    fingerprintMaximumNativeFrameGapSeconds
                        ?? maximumNativeFrameGapSeconds,
                nativeFrameGapMultiplier: nativeFrameGapMultiplier,
                nativeFrameJitterAllowanceSeconds:
                    nativeFrameJitterAllowanceSeconds,
                maximumAnalysisPixelCount:
                    fingerprintMaximumAnalysisPixelCount
                        ?? maximumAnalysisPixelCount,
                maximumAnalysisDimensionPixels:
                    fingerprintMaximumAnalysisDimensionPixels
                        ?? maximumAnalysisDimensionPixels,
                maximumAnalysisAspectRatio:
                    fingerprintMaximumAnalysisAspectRatio
                        ?? maximumAnalysisAspectRatio,
                maximumRetainedPixelBufferBytes:
                    fingerprintMaximumRetainedPixelBufferBytes
                        ?? maximumRetainedPixelBufferBytes,
                maximumRetainedPixelBufferCount:
                    fingerprintMaximumRetainedPixelBufferCount
                        ?? CameraAnalysisBufferBudget.maximumRetainedBufferCount,
                appearanceBoxAverageSizePixels:
                    fingerprintAppearanceBoxAverageSizePixels
                        ?? appearanceBoxAverageSizePixels,
                appearanceBoxAverageSpacingPixels:
                    fingerprintAppearanceBoxAverageSpacingPixels
                        ?? appearanceBoxAverageSpacingPixels,
                registrationCorrelationSampleSpacingPixels:
                    fingerprintRegistrationCorrelationSampleSpacingPixels
                        ?? registrationCorrelationSampleSpacingPixels,
                registrationPeakSearchRadiusPixels:
                    fingerprintRegistrationPeakSearchRadiusPixels
                        ?? registrationPeakSearchRadiusPixels,
                registrationPeakMinimumSeparationPixels:
                    fingerprintRegistrationPeakMinimumSeparationPixels
                        ?? registrationPeakMinimumSeparationPixels,
                registrationAliasMinimumOverlapPairCount:
                    fingerprintRegistrationAliasMinimumOverlapPairCount
                        ?? CameraRegistrationPeakAnalyzer
                            .aliasMinimumOverlapPairCount,
                registrationAliasSharedDomainSideSamples:
                    fingerprintRegistrationAliasSharedDomainSideSamples
                        ?? CameraRegistrationPeakAnalyzer
                            .aliasSharedDomainSideSamples,
                registrationAliasTailPairCount:
                    fingerprintRegistrationAliasTailPairCount
                        ?? CameraRegistrationPeakAnalyzer.aliasTailPairCount,
                maximumRegistrationCorrelationPairCountPerTile:
                    fingerprintMaximumRegistrationCorrelationPairCountPerTile
                        ?? CameraRegistrationPeakAnalyzer
                            .maximumCorrelationPairCountPerTile,
                maximumScanDurationSeconds:
                    CameraMotionScanBudgetPolicy.maximumRangeDurationSeconds,
                maximumNativeSampleCount:
                    CameraMotionScanBudgetPolicy.maximumNativeSampleCount,
                personBoxInflationFraction: Double(
                    fingerprintPersonBoxInflationFraction
                        ?? personBoxInflationFraction
                ),
                minimumTileAreaFraction: 0.01,
                minimumBackgroundFraction: 0.20,
                minimumRegistrationQuality:
                    fingerprintMinimumRegistrationQuality
                        ?? minimumRegistrationQuality,
                minimumRegistrationPeakCorrelation:
                    fingerprintMinimumRegistrationPeakCorrelation
                        ?? minimumRegistrationPeakCorrelation,
                minimumTileCount: 3,
                minimumStructureTensorEigenvalue: 0.0005,
                minimumRegistrationUniqueness:
                    fingerprintMinimumRegistrationUniqueness
                        ?? minimumRegistrationUniqueness,
                maximumPostWarpMeanAbsoluteError: 20,
                maximumFitResidualPixels: 2,
                minimumSpatialEigenvalueFraction: 0.001,
                derivativeWindowDomainSeconds: derivativeWindowDomainSeconds,
                nativeFrameIntervalDomainSeconds:
                    nativeFrameIntervalDomainSeconds
            )
        } else {
            fingerprint = nil
        }
        return CameraMotionVideoAnalyzer.Configuration(
            provisionalStaticTranslation: provisionalStaticTranslation,
            provisionalMovingTranslation: 0.02,
            provisionalStaticRotationRadians: 0.01,
            provisionalMovingRotationRadians: 0.02,
            provisionalStaticScaleFraction: 0.01,
            provisionalMovingScaleFraction: 0.02,
            maximumNativeFrameGapSeconds: maximumNativeFrameGapSeconds,
            nativeFrameGapMultiplier: nativeFrameGapMultiplier,
            nativeFrameJitterAllowanceSeconds:
                nativeFrameJitterAllowanceSeconds,
            maximumAnalysisPixelCount: maximumAnalysisPixelCount,
            maximumAnalysisDimensionPixels: maximumAnalysisDimensionPixels,
            maximumAnalysisAspectRatio: maximumAnalysisAspectRatio,
            maximumRetainedPixelBufferBytes:
                maximumRetainedPixelBufferBytes,
            appearanceBoxAverageSizePixels: appearanceBoxAverageSizePixels,
            appearanceBoxAverageSpacingPixels:
                appearanceBoxAverageSpacingPixels,
            registrationCorrelationSampleSpacingPixels:
                registrationCorrelationSampleSpacingPixels,
            registrationPeakSearchRadiusPixels:
                registrationPeakSearchRadiusPixels,
            registrationPeakMinimumSeparationPixels:
                registrationPeakMinimumSeparationPixels,
            personBoxInflationFraction: personBoxInflationFraction,
            minimumTileAreaFraction: 0.01,
            minimumBackgroundFraction: 0.20,
            minimumRegistrationQuality: minimumRegistrationQuality,
            minimumRegistrationPeakCorrelation:
                minimumRegistrationPeakCorrelation,
            tileFit: .init(
                minimumTileCount: 3,
                minimumStructureTensorEigenvalue: 0.0005,
                minimumRegistrationUniqueness: minimumRegistrationUniqueness,
                maximumPostWarpMeanAbsoluteError: 20,
                maximumFitResidualPixels: 2,
                minimumSpatialEigenvalueFraction: 0.001
            ),
            calibrationProfileID: profileID,
            calibrationFrameSizePixels: frameSizePixels,
            calibrationDerivativeWindowDomainSeconds:
                derivativeWindowDomainSeconds,
            calibrationNativeFrameIntervalDomainSeconds:
                nativeFrameIntervalDomainSeconds,
            calibrationFingerprint: fingerprint
        )
    }

    private func isReady(
        _ configuration: CameraMotionVideoAnalyzer.Configuration
    ) -> Bool {
        CameraMotionVideoAnalyzer(configuration: configuration)
            .isVersionedCalibrationReady
    }

    private func cadenceAssessment(
        intervals: [TimeInterval],
        derivativeWindowSeconds: TimeInterval
    ) -> CameraMotionCadenceAssessment? {
        CameraMotionCadencePolicy.assess(
            nativeFrameIntervals: intervals,
            derivativeWindowSeconds: derivativeWindowSeconds,
            hardMaximumGapSeconds: 0.075,
            gapMultiplier: 1.5,
            jitterAllowanceSeconds: 0.005
        )
    }

    private func assertSourceBoundsFillOrFitRender(
        naturalSize: CGSize,
        geometry: CameraAnalysisRenderGeometry,
        file: StaticString = #filePath,
        line: UInt = #line
    ) {
        let bounds = CGRect(origin: .zero, size: naturalSize)
            .applying(geometry.sourceToRenderTransform)
            .standardized
        XCTAssertEqual(bounds.minX, 0, accuracy: 1e-9, file: file, line: line)
        XCTAssertEqual(bounds.minY, 0, accuracy: 1e-9, file: file, line: line)
        XCTAssertLessThanOrEqual(
            bounds.maxX,
            CGFloat(geometry.pixelDimensions.x) + 1e-9,
            file: file,
            line: line
        )
        XCTAssertLessThanOrEqual(
            bounds.maxY,
            CGFloat(geometry.pixelDimensions.y) + 1e-9,
            file: file,
            line: line
        )
        XCTAssertTrue(
            abs(bounds.width - CGFloat(geometry.pixelDimensions.x)) < 1e-9
                || abs(bounds.height - CGFloat(geometry.pixelDimensions.y)) < 1e-9,
            "uniform scale must fill at least one render axis",
            file: file,
            line: line
        )
    }
}

private struct DefaultCalibrationReadinessAnalyzer: CameraMotionAnalyzing {
    func analyzeVideo(
        at url: URL,
        range: CameraMotionAnalysisRange,
        derivativeWindowSeconds: TimeInterval
    ) async throws -> CameraReferenceState {
        .unmeasured
    }
}
