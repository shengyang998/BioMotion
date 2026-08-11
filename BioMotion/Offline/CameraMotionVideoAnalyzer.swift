import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import Vision

/// Complete identity of the adapter configuration that produced a versioned
/// calibration profile. The profile ID and frame domain are deliberately part
/// of the value: copying a label onto changed thresholds, masking, cadence,
/// registration, or resource bounds must not make that configuration ready.
struct CameraMotionCalibrationFingerprint: Equatable, Sendable {
    let profileID: String
    let analysisFrameSizePixels: SIMD2<Int>
    let adapterRevision: Int
    let humanRectanglesRequestRevision: Int
    let translationalRegistrationRequestRevision: Int
    let provisionalStaticTranslation: Double
    let provisionalMovingTranslation: Double
    let provisionalStaticRotationRadians: Double
    let provisionalMovingRotationRadians: Double
    let provisionalStaticScaleFraction: Double
    let provisionalMovingScaleFraction: Double
    let maximumNativeFrameGapSeconds: TimeInterval
    let nativeFrameGapMultiplier: Double
    let nativeFrameJitterAllowanceSeconds: TimeInterval
    let maximumAnalysisPixelCount: Int
    let maximumAnalysisDimensionPixels: Int
    let maximumAnalysisAspectRatio: Double
    let maximumRetainedPixelBufferBytes: Int
    let maximumRetainedPixelBufferCount: Int
    let appearanceBoxAverageSizePixels: Int
    let appearanceBoxAverageSpacingPixels: Int
    let registrationCorrelationSampleSpacingPixels: Int
    let registrationPeakSearchRadiusPixels: Int
    let registrationPeakMinimumSeparationPixels: Int
    let registrationAliasMinimumOverlapPairCount: Int
    let registrationAliasSharedDomainSideSamples: Int
    let registrationAliasTailPairCount: Int
    let maximumRegistrationCorrelationPairCountPerTile: Int
    let maximumScanDurationSeconds: TimeInterval
    let maximumNativeSampleCount: Int
    let personBoxInflationFraction: Double
    let minimumTileAreaFraction: Double
    let minimumBackgroundFraction: Double
    let minimumRegistrationQuality: Double
    let minimumRegistrationPeakCorrelation: Double
    let minimumTileCount: Int
    let minimumStructureTensorEigenvalue: Double
    let minimumRegistrationUniqueness: Double
    let maximumPostWarpMeanAbsoluteError: Double
    let maximumFitResidualPixels: Double
    let minimumSpatialEigenvalueFraction: Double
    let derivativeWindowDomainSeconds: ClosedRange<TimeInterval>
    let nativeFrameIntervalDomainSeconds: ClosedRange<TimeInterval>
}

/// Native-PTS, background-only camera motion measurement for imported video.
///
/// This reader is intentionally independent of `FrameSource`: pose sampling can
/// be sparse, while a camera pan must be observed at the source track's actual
/// cadence. A bounded video composition normalizes the selected track into
/// upright image space before every Vision request, and actual sample PTS are
/// carried verbatim into the reducer.
final class CameraMotionVideoAnalyzer: CameraMotionAnalyzing, @unchecked Sendable {
    /// Bump whenever reader timing, Vision requests, texture screening, tile
    /// registration, fit geometry, or reduction semantics change. A calibrated
    /// fingerprint from an older implementation then fails admission even when
    /// all numeric knobs happen to be unchanged.
    static let calibrationAdapterRevision = 3
    static let humanRectanglesRequestRevision =
        VNDetectHumanRectanglesRequestRevision2
    static let translationalRegistrationRequestRevision =
        VNTranslationalImageRegistrationRequestRevision1

    /// Revision availability is a runtime property of Vision. Calibration must
    /// stop before opening media when either exact algorithm revision is absent;
    /// silently accepting Vision's current/default revision would invalidate the
    /// fixture-derived fingerprint.
    static var pinnedVisionRequestRevisionsAreSupported: Bool {
        VNDetectHumanRectanglesRequest.supportedRevisions.contains(
            humanRectanglesRequestRevision
        ) && VNTranslationalImageRegistrationRequest.supportedRevisions.contains(
            translationalRegistrationRequestRevision
        )
    }

    struct Configuration: Equatable, Sendable {
        let provisionalStaticTranslation: Double
        let provisionalMovingTranslation: Double
        let provisionalStaticRotationRadians: Double
        let provisionalMovingRotationRadians: Double
        let provisionalStaticScaleFraction: Double
        let provisionalMovingScaleFraction: Double
        let maximumNativeFrameGapSeconds: TimeInterval
        let nativeFrameGapMultiplier: Double
        let nativeFrameJitterAllowanceSeconds: TimeInterval
        let maximumAnalysisPixelCount: Int
        let maximumAnalysisDimensionPixels: Int
        let maximumAnalysisAspectRatio: Double
        let maximumRetainedPixelBufferBytes: Int
        let appearanceBoxAverageSizePixels: Int
        let appearanceBoxAverageSpacingPixels: Int
        let registrationCorrelationSampleSpacingPixels: Int
        let registrationPeakSearchRadiusPixels: Int
        let registrationPeakMinimumSeparationPixels: Int
        let personBoxInflationFraction: CGFloat
        let minimumTileAreaFraction: CGFloat
        let minimumBackgroundFraction: Double
        let minimumRegistrationQuality: Double
        let minimumRegistrationPeakCorrelation: Double
        let tileFit: CameraTileMotionFitter.Configuration
        /// Must remain nil until the same implementation and exact analysis
        /// dimensions pass tripod/static controls and held-out pan/tilt/roll/
        /// push-in/zoom fixtures. The old OpenCV probe and `0.05g` do not
        /// calibrate Vision image displacement.
        let calibrationProfileID: String?
        let calibrationFrameSizePixels: SIMD2<Int>?
        let calibrationDerivativeWindowDomainSeconds:
            ClosedRange<TimeInterval>?
        let calibrationNativeFrameIntervalDomainSeconds:
            ClosedRange<TimeInterval>?
        let calibrationFingerprint: CameraMotionCalibrationFingerprint?

        static let production = Configuration(
            provisionalStaticTranslation: 0.01,
            provisionalMovingTranslation: 0.02,
            provisionalStaticRotationRadians: 0.01,
            provisionalMovingRotationRadians: 0.02,
            provisionalStaticScaleFraction: 0.01,
            provisionalMovingScaleFraction: 0.02,
            maximumNativeFrameGapSeconds: 0.075,
            nativeFrameGapMultiplier: 1.5,
            nativeFrameJitterAllowanceSeconds: 0.005,
            maximumAnalysisPixelCount: 1_920 * 1_080,
            maximumAnalysisDimensionPixels: 4_096,
            maximumAnalysisAspectRatio: 4,
            maximumRetainedPixelBufferBytes: 64 * 1_024 * 1_024,
            appearanceBoxAverageSizePixels: 8,
            appearanceBoxAverageSpacingPixels: 8,
            registrationCorrelationSampleSpacingPixels: 32,
            registrationPeakSearchRadiusPixels: 48,
            registrationPeakMinimumSeparationPixels: 16,
            personBoxInflationFraction: 0.03,
            minimumTileAreaFraction: 0.01,
            minimumBackgroundFraction: 0.20,
            minimumRegistrationQuality: 0.5,
            minimumRegistrationPeakCorrelation: 0.5,
            tileFit: .init(
                minimumTileCount: 3,
                minimumStructureTensorEigenvalue: 0.0005,
                minimumRegistrationUniqueness: 0.05,
                maximumPostWarpMeanAbsoluteError: 20,
                maximumFitResidualPixels: 2,
                minimumSpatialEigenvalueFraction: 0.001
            ),
            calibrationProfileID: nil,
            calibrationFrameSizePixels: nil,
            calibrationDerivativeWindowDomainSeconds: nil,
            calibrationNativeFrameIntervalDomainSeconds: nil,
            calibrationFingerprint: nil
        )
    }

    private struct Snapshot: @unchecked Sendable {
        let pixelBuffer: CVReadOnlyPixelBuffer
        let timestamp: TimeInterval
        let personBoxes: [CGRect]
    }

    private enum AnalysisError: Error {
        case indeterminate(CameraReferenceIndeterminateReason)
    }

    private let configuration: Configuration

    init(configuration: Configuration = .production) {
        self.configuration = configuration
    }

    var isVersionedCalibrationReady: Bool {
        guard Self.pinnedVisionRequestRevisionsAreSupported,
              configurationIsStructurallyValid else { return false }
        let currentFingerprint = makeCurrentCalibrationFingerprint()
        return CameraCalibrationReadinessPolicy.isReady(
            profileID: configuration.calibrationProfileID,
            frameSizePixels: configuration.calibrationFrameSizePixels,
            maximumAnalysisPixelCount: configuration.maximumAnalysisPixelCount,
            maximumAnalysisDimensionPixels:
                configuration.maximumAnalysisDimensionPixels,
            maximumAnalysisAspectRatio:
                configuration.maximumAnalysisAspectRatio,
            maximumRetainedPixelBufferBytes:
                configuration.maximumRetainedPixelBufferBytes,
            appearanceBoxAverageSizePixels:
                configuration.appearanceBoxAverageSizePixels,
            appearanceBoxAverageSpacingPixels:
                configuration.appearanceBoxAverageSpacingPixels,
            registrationCorrelationSampleSpacingPixels:
                configuration.registrationCorrelationSampleSpacingPixels,
            registrationPeakSearchRadiusPixels:
                configuration.registrationPeakSearchRadiusPixels,
            registrationPeakMinimumSeparationPixels:
                configuration.registrationPeakMinimumSeparationPixels,
            minimumTileAreaFraction:
                Double(configuration.minimumTileAreaFraction),
            derivativeWindowDomainSeconds:
                configuration.calibrationDerivativeWindowDomainSeconds,
            nativeFrameIntervalDomainSeconds:
                configuration.calibrationNativeFrameIntervalDomainSeconds,
            claimedFingerprintMatchesConfiguration:
                configuration.calibrationFingerprint != nil
                    && configuration.calibrationFingerprint == currentFingerprint
        )
    }

    private var configurationIsStructurallyValid: Bool {
        let tile = configuration.tileFit
        return configuration.provisionalStaticTranslation.isFinite
            && configuration.provisionalStaticTranslation >= 0
            && configuration.provisionalMovingTranslation.isFinite
            && configuration.provisionalMovingTranslation
                > configuration.provisionalStaticTranslation
            && configuration.provisionalStaticRotationRadians.isFinite
            && configuration.provisionalStaticRotationRadians >= 0
            && configuration.provisionalMovingRotationRadians.isFinite
            && configuration.provisionalMovingRotationRadians
                > configuration.provisionalStaticRotationRadians
            && configuration.provisionalStaticScaleFraction.isFinite
            && configuration.provisionalStaticScaleFraction >= 0
            && configuration.provisionalMovingScaleFraction.isFinite
            && configuration.provisionalMovingScaleFraction
                > configuration.provisionalStaticScaleFraction
            && configuration.maximumNativeFrameGapSeconds.isFinite
            && configuration.maximumNativeFrameGapSeconds > 0
            && configuration.nativeFrameGapMultiplier.isFinite
            && configuration.nativeFrameGapMultiplier >= 1
            && configuration.nativeFrameJitterAllowanceSeconds.isFinite
            && configuration.nativeFrameJitterAllowanceSeconds >= 0
            && configuration.maximumAnalysisPixelCount > 0
            && configuration.maximumAnalysisDimensionPixels >= 2
            && configuration.maximumAnalysisAspectRatio.isFinite
            && configuration.maximumAnalysisAspectRatio >= 1
            && configuration.maximumRetainedPixelBufferBytes > 0
            && configuration.appearanceBoxAverageSizePixels > 0
            && configuration.appearanceBoxAverageSpacingPixels
                >= configuration.appearanceBoxAverageSizePixels
            && configuration.registrationCorrelationSampleSpacingPixels
                >= configuration.appearanceBoxAverageSpacingPixels
            && configuration.registrationCorrelationSampleSpacingPixels
                .isMultiple(of: configuration.appearanceBoxAverageSpacingPixels)
            && configuration.registrationCorrelationSampleSpacingPixels
                / configuration.appearanceBoxAverageSpacingPixels <= 64
            && configuration.registrationPeakSearchRadiusPixels
                >= configuration.appearanceBoxAverageSpacingPixels
            && configuration.registrationPeakSearchRadiusPixels
                .isMultiple(of: configuration.appearanceBoxAverageSpacingPixels)
            && configuration.registrationPeakSearchRadiusPixels
                / configuration.appearanceBoxAverageSpacingPixels <= 64
            && configuration.registrationPeakMinimumSeparationPixels
                >= configuration.appearanceBoxAverageSpacingPixels
            && configuration.registrationPeakMinimumSeparationPixels
                <= configuration.registrationPeakSearchRadiusPixels
            && configuration.registrationPeakMinimumSeparationPixels
                .isMultiple(of: configuration.appearanceBoxAverageSpacingPixels)
            && configuration.personBoxInflationFraction.isFinite
            && (0..<0.5).contains(configuration.personBoxInflationFraction)
            && configuration.minimumTileAreaFraction.isFinite
            && (0...1).contains(configuration.minimumTileAreaFraction)
            && configuration.minimumBackgroundFraction.isFinite
            && (0...1).contains(configuration.minimumBackgroundFraction)
            && configuration.minimumRegistrationQuality.isFinite
            && (0...1).contains(configuration.minimumRegistrationQuality)
            && configuration.minimumRegistrationPeakCorrelation.isFinite
            && (0...1).contains(
                configuration.minimumRegistrationPeakCorrelation
            )
            && tile.minimumTileCount >= 3
            && tile.minimumStructureTensorEigenvalue.isFinite
            && tile.minimumStructureTensorEigenvalue >= 0
            && tile.minimumRegistrationUniqueness.isFinite
            && (0...1).contains(tile.minimumRegistrationUniqueness)
            && tile.maximumPostWarpMeanAbsoluteError.isFinite
            && tile.maximumPostWarpMeanAbsoluteError > 0
            && tile.maximumFitResidualPixels.isFinite
            && tile.maximumFitResidualPixels > 0
            && tile.minimumSpatialEigenvalueFraction.isFinite
            && tile.minimumSpatialEigenvalueFraction > 0
    }

    private func makeCurrentCalibrationFingerprint()
        -> CameraMotionCalibrationFingerprint? {
        guard let profileID = configuration.calibrationProfileID,
              let frameSize = configuration.calibrationFrameSizePixels,
              let derivativeWindowDomain =
                configuration.calibrationDerivativeWindowDomainSeconds,
              let nativeFrameIntervalDomain =
                configuration.calibrationNativeFrameIntervalDomainSeconds else {
            return nil
        }
        let tileFit = configuration.tileFit
        return CameraMotionCalibrationFingerprint(
            profileID: profileID,
            analysisFrameSizePixels: frameSize,
            adapterRevision: Self.calibrationAdapterRevision,
            humanRectanglesRequestRevision:
                Self.humanRectanglesRequestRevision,
            translationalRegistrationRequestRevision:
                Self.translationalRegistrationRequestRevision,
            provisionalStaticTranslation:
                configuration.provisionalStaticTranslation,
            provisionalMovingTranslation:
                configuration.provisionalMovingTranslation,
            provisionalStaticRotationRadians:
                configuration.provisionalStaticRotationRadians,
            provisionalMovingRotationRadians:
                configuration.provisionalMovingRotationRadians,
            provisionalStaticScaleFraction:
                configuration.provisionalStaticScaleFraction,
            provisionalMovingScaleFraction:
                configuration.provisionalMovingScaleFraction,
            maximumNativeFrameGapSeconds:
                configuration.maximumNativeFrameGapSeconds,
            nativeFrameGapMultiplier: configuration.nativeFrameGapMultiplier,
            nativeFrameJitterAllowanceSeconds:
                configuration.nativeFrameJitterAllowanceSeconds,
            maximumAnalysisPixelCount: configuration.maximumAnalysisPixelCount,
            maximumAnalysisDimensionPixels:
                configuration.maximumAnalysisDimensionPixels,
            maximumAnalysisAspectRatio:
                configuration.maximumAnalysisAspectRatio,
            maximumRetainedPixelBufferBytes:
                configuration.maximumRetainedPixelBufferBytes,
            maximumRetainedPixelBufferCount:
                CameraAnalysisBufferBudget.maximumRetainedBufferCount,
            appearanceBoxAverageSizePixels:
                configuration.appearanceBoxAverageSizePixels,
            appearanceBoxAverageSpacingPixels:
                configuration.appearanceBoxAverageSpacingPixels,
            registrationCorrelationSampleSpacingPixels:
                configuration.registrationCorrelationSampleSpacingPixels,
            registrationPeakSearchRadiusPixels:
                configuration.registrationPeakSearchRadiusPixels,
            registrationPeakMinimumSeparationPixels:
                configuration.registrationPeakMinimumSeparationPixels,
            registrationAliasMinimumOverlapPairCount:
                CameraRegistrationPeakAnalyzer.aliasMinimumOverlapPairCount,
            registrationAliasSharedDomainSideSamples:
                CameraRegistrationPeakAnalyzer.aliasSharedDomainSideSamples,
            registrationAliasTailPairCount:
                CameraRegistrationPeakAnalyzer.aliasTailPairCount,
            maximumRegistrationCorrelationPairCountPerTile:
                CameraRegistrationPeakAnalyzer
                    .maximumCorrelationPairCountPerTile,
            maximumScanDurationSeconds:
                CameraMotionScanBudgetPolicy.maximumRangeDurationSeconds,
            maximumNativeSampleCount:
                CameraMotionScanBudgetPolicy.maximumNativeSampleCount,
            personBoxInflationFraction:
                Double(configuration.personBoxInflationFraction),
            minimumTileAreaFraction:
                Double(configuration.minimumTileAreaFraction),
            minimumBackgroundFraction: configuration.minimumBackgroundFraction,
            minimumRegistrationQuality:
                configuration.minimumRegistrationQuality,
            minimumRegistrationPeakCorrelation:
                configuration.minimumRegistrationPeakCorrelation,
            minimumTileCount: tileFit.minimumTileCount,
            minimumStructureTensorEigenvalue:
                tileFit.minimumStructureTensorEigenvalue,
            minimumRegistrationUniqueness:
                tileFit.minimumRegistrationUniqueness,
            maximumPostWarpMeanAbsoluteError:
                tileFit.maximumPostWarpMeanAbsoluteError,
            maximumFitResidualPixels: tileFit.maximumFitResidualPixels,
            minimumSpatialEigenvalueFraction:
                tileFit.minimumSpatialEigenvalueFraction,
            derivativeWindowDomainSeconds: derivativeWindowDomain,
            nativeFrameIntervalDomainSeconds: nativeFrameIntervalDomain
        )
    }

    func analyzeVideo(
        at url: URL,
        range: CameraMotionAnalysisRange,
        derivativeWindowSeconds: TimeInterval
    ) async throws -> CameraReferenceState {
        try Task.checkCancellation()
        guard isVersionedCalibrationReady,
              let fingerprint = configuration.calibrationFingerprint,
              fingerprint.derivativeWindowDomainSeconds
                .contains(derivativeWindowSeconds) else {
            // This public boundary remains safe even when a caller bypasses the
            // runner's cheap admission policy. No reader or Vision work is
            // justified when the build has no profile for this runtime window.
            return .calibrationUnavailable
        }
        do {
            return try await analyze(
                at: url,
                range: range,
                derivativeWindowSeconds: derivativeWindowSeconds
            )
        } catch is CancellationError {
            throw CancellationError()
        } catch AnalysisError.indeterminate(let reason) {
            return .indeterminate(reason)
        } catch {
            return .indeterminate(.videoReadFailed)
        }
    }

    private func analyze(
        at url: URL,
        range: CameraMotionAnalysisRange,
        derivativeWindowSeconds: TimeInterval
    ) async throws -> CameraReferenceState {
        // Cancellation outranks every deterministic refusal, including an
        // over-budget request, so a cancelled predecessor cannot publish a new
        // clip-level state when it resumes.
        try Task.checkCancellation()
        guard range.startSeconds.isFinite, range.endSeconds.isFinite,
              range.startSeconds >= 0, range.endSeconds > range.startSeconds,
              derivativeWindowSeconds.isFinite,
              derivativeWindowSeconds > 0 else {
            throw AnalysisError.indeterminate(.invalidMeasurement)
        }
        if let reason = CameraMotionScanBudgetPolicy.rangeIndeterminateReason(range) {
            throw AnalysisError.indeterminate(reason)
        }

        let asset = AVURLAsset(url: url)
        let tracks = try await asset.loadTracks(withMediaType: .video)
        try Task.checkCancellation()
        if let reason = CameraMotionVideoTrackPolicy.indeterminateReason(
            videoTrackCount: tracks.count
        ) {
            throw AnalysisError.indeterminate(reason)
        }
        guard let primaryTrack = tracks.first else {
            throw AnalysisError.indeterminate(.videoReadFailed)
        }

        let readerRange = CMTimeRange(
            start: CMTime(seconds: range.startSeconds, preferredTimescale: 60_000),
            duration: CMTime(
                seconds: range.endSeconds - range.startSeconds,
                preferredTimescale: 60_000
            )
        )
        let segments = try await primaryTrack.load(.segments)
        try Task.checkCancellation()
        let containsEmptyEdit = segments.contains { segment in
            guard segment.isEmpty else { return false }
            let overlap = CMTimeRangeGetIntersection(
                segment.timeMapping.target,
                otherRange: readerRange
            )
            return overlap.isValid && CMTimeCompare(overlap.duration, .zero) > 0
        }
        guard !containsEmptyEdit else {
            throw AnalysisError.indeterminate(.incompleteRangeCoverage)
        }

        let naturalSize = try await primaryTrack.load(.naturalSize)
        try Task.checkCancellation()
        let formatDescriptions = try await primaryTrack.load(.formatDescriptions)
        try Task.checkCancellation()
        let formatGeometries = formatDescriptions.compactMap {
            description -> CameraVideoFormatGeometry? in
            guard CMFormatDescriptionGetMediaType(description)
                    == kCMMediaType_Video else { return nil }
            let dimensions = CMVideoFormatDescriptionGetDimensions(description)
            guard dimensions.width > 0, dimensions.height > 0 else { return nil }
            return CameraVideoFormatGeometry(
                encodedDimensions: SIMD2(
                    Int(dimensions.width), Int(dimensions.height)
                ),
                cleanAperture: CMVideoFormatDescriptionGetCleanAperture(
                    description,
                    originIsAtTopLeft: true
                ),
                pixelAspectAdjustedDimensions:
                    CMVideoFormatDescriptionGetPresentationDimensions(
                        description,
                        usePixelAspectRatio: true,
                        useCleanAperture: false
                    )
            )
        }
        guard formatGeometries.count == formatDescriptions.count,
              CameraVideoFormatGeometryPolicy.isSupported(
                  naturalSize: naturalSize,
                  formatGeometries: formatGeometries
              ) else {
            throw AnalysisError.indeterminate(.analysisResolutionUnsupported)
        }
        let preferredTransform = try await primaryTrack.load(.preferredTransform)
        try Task.checkCancellation()
        let minimumFrameDuration = try await primaryTrack.load(.minFrameDuration)
        try Task.checkCancellation()

        guard let renderGeometry = CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: naturalSize,
            preferredTransform: preferredTransform,
            maximumPixelCount: configuration.maximumAnalysisPixelCount,
            maximumDimensionPixels:
                configuration.maximumAnalysisDimensionPixels,
            maximumAspectRatio: configuration.maximumAnalysisAspectRatio
        ) else {
            throw AnalysisError.indeterminate(.analysisResolutionUnsupported)
        }
        let analysisFrameSize = renderGeometry.pixelDimensions
        guard let calibratedFrameSize = configuration.calibrationFrameSizePixels,
              let calibrationFingerprint = configuration.calibrationFingerprint,
              analysisFrameSize == calibratedFrameSize,
              calibrationFingerprint.analysisFrameSizePixels
                == analysisFrameSize else {
            // Render-pixel thresholds, sampling spacings, and search radii are
            // calibrated for one exact raster. Reject before reader/Vision work.
            throw AnalysisError.indeterminate(.analysisResolutionUnsupported)
        }

        var layerConfiguration = AVVideoCompositionLayerInstruction.Configuration(
            assetTrack: primaryTrack
        )
        layerConfiguration.setTransform(
            renderGeometry.sourceToRenderTransform,
            at: readerRange.start
        )
        let layerInstruction = AVVideoCompositionLayerInstruction(
            configuration: layerConfiguration
        )
        let instruction = AVVideoCompositionInstruction(configuration: .init(
            layerInstructions: [layerInstruction],
            timeRange: readerRange
        ))
        let fallbackFrameDuration = minimumFrameDuration.isNumeric
            && minimumFrameDuration > .zero
            ? minimumFrameDuration
            : CMTime(value: 1, timescale: 30)
        let videoConfiguration = AVVideoComposition.Configuration(
            frameDuration: fallbackFrameDuration,
            instructions: [instruction],
            renderSize: CGSize(
                width: analysisFrameSize.x,
                height: analysisFrameSize.y
            ),
            sourceTrackIDForFrameTiming: primaryTrack.trackID
        )
        let output = AVAssetReaderVideoCompositionOutput(
            videoTracks: [primaryTrack],
            videoSettings: [
                kCVPixelBufferPixelFormatTypeKey as String:
                    Int(kCVPixelFormatType_32BGRA),
            ]
        )
        output.videoComposition = AVVideoComposition(configuration: videoConfiguration)

        let reader = try AVAssetReader(asset: asset)
        reader.timeRange = readerRange
        let provider = reader.outputProvider(for: output)
        try reader.start()
        defer {
            // The provider itself has no cancel operation and `next()` does not
            // automatically stop when the Swift task is cancelled. Cancellation
            // checks therefore bracket every await and Vision call below. Only
            // this reader task calls cancelReading, avoiding concurrent access.
            if reader.status == .reading { reader.cancelReading() }
        }

        var previous: Snapshot?
        var anchors: [Snapshot] = []
        var observations: [CameraMotionObservation] = []
        var firstSampleTimestamp: TimeInterval?
        var lastSampleTimestamp: TimeInterval?
        var lastSampleDuration: TimeInterval?
        var nativeSampleCount = 0
        var nativeSampleTimestamps: [TimeInterval] = []
        var nativeSampleDurations: [TimeInterval] = []
        nativeSampleTimestamps.reserveCapacity(
            min(CameraMotionScanBudgetPolicy.maximumNativeSampleCount, 1_000)
        )
        nativeSampleDurations.reserveCapacity(
            min(CameraMotionScanBudgetPolicy.maximumNativeSampleCount, 1_000)
        )
        let anchorStride = derivativeWindowSeconds / 2

        while true {
            try Task.checkCancellation()
            let nextSample = try await provider.next()
            try Task.checkCancellation()
            guard let sample = nextSample else { break }
            nativeSampleCount += 1
            if let reason = CameraMotionScanBudgetPolicy.sampleIndeterminateReason(
                nativeSampleCount: nativeSampleCount
            ) {
                // The 1,001st sample is fetched only to distinguish an exactly
                // full budget from an over-budget stream. Reject it before pixel
                // conversion, person detection, or any registration request.
                throw AnalysisError.indeterminate(reason)
            }
            guard case .pixelBuffer(let readOnlyBuffer) = sample.content else {
                throw AnalysisError.indeterminate(.videoReadFailed)
            }
            let timestamp = sample.presentationTimeStamp.seconds
            guard timestamp.isFinite else {
                throw AnalysisError.indeterminate(.invalidMeasurement)
            }
            if firstSampleTimestamp == nil { firstSampleTimestamp = timestamp }
            lastSampleTimestamp = timestamp
            nativeSampleTimestamps.append(timestamp)
            let mediaDuration = sample.duration
            let durationSeconds = mediaDuration.seconds
            guard mediaDuration.isNumeric,
                  mediaDuration > .zero,
                  durationSeconds.isFinite,
                  durationSeconds > 0 else {
                throw AnalysisError.indeterminate(.incompleteRangeCoverage)
            }
            lastSampleDuration = durationSeconds
            nativeSampleDurations.append(durationSeconds)

            let boxes: [CGRect]
            do {
                boxes = try readOnlyBuffer.withUnsafeBuffer { buffer in
                    try Task.checkCancellation()
                    guard CVPixelBufferGetWidth(buffer) == analysisFrameSize.x,
                          CVPixelBufferGetHeight(buffer) == analysisFrameSize.y,
                          CVPixelBufferGetPixelFormatType(buffer)
                            == kCVPixelFormatType_32BGRA,
                          CameraAnalysisBufferBudget.isSupported(
                              widthPixels: CVPixelBufferGetWidth(buffer),
                              heightPixels: CVPixelBufferGetHeight(buffer),
                              bytesPerRow: CVPixelBufferGetBytesPerRow(buffer),
                              maximumPixelCount:
                                  configuration.maximumAnalysisPixelCount,
                              maximumDimensionPixels:
                                  configuration.maximumAnalysisDimensionPixels,
                              maximumAspectRatio:
                                  configuration.maximumAnalysisAspectRatio,
                              maximumRetainedBytes:
                                  configuration.maximumRetainedPixelBufferBytes
                          ) else {
                        throw AnalysisError.indeterminate(
                            .analysisResolutionUnsupported
                        )
                    }
                    do {
                        let detected = try Self.personBoxes(in: buffer)
                        try Task.checkCancellation()
                        return detected
                    } catch is CancellationError {
                        throw CancellationError()
                    } catch {
                        throw AnalysisError.indeterminate(.personDetectionFailed)
                    }
                }
            } catch is CancellationError {
                throw CancellationError()
            } catch let error as AnalysisError {
                throw error
            }
            guard !boxes.isEmpty else {
                throw AnalysisError.indeterminate(.personDetectionFailed)
            }
            let current = Snapshot(
                pixelBuffer: readOnlyBuffer,
                timestamp: timestamp,
                personBoxes: boxes
            )

            if let previous {
                try Task.checkCancellation()
                let step = try estimate(reference: previous, target: current)

                anchors.removeAll {
                    current.timestamp - $0.timestamp
                        > derivativeWindowSeconds + 1e-9
                }
                if anchors.isEmpty { anchors.append(previous) }

                var anchored: [CameraAnchoredMotionEstimate] = []
                anchored.reserveCapacity(anchors.count)
                for anchor in anchors {
                    try Task.checkCancellation()
                    let fit = try estimate(reference: anchor, target: current)
                    anchored.append(CameraAnchoredMotionEstimate(
                        baselineTimestamp: anchor.timestamp,
                        fit: fit
                    ))
                }
                guard let peaks = CameraAnchorPeakSelector.select(anchored) else {
                    throw AnalysisError.indeterminate(.registrationFailed)
                }
                let allFits = anchored.map(\.fit) + [step]
                let quality = allFits.map(\.registrationQuality).min() ?? 0
                let background = allFits.map(\.usableBackgroundFraction).min() ?? 0
                observations.append(CameraMotionObservation(
                    startTimestamp: previous.timestamp,
                    endTimestamp: current.timestamp,
                    translationWindowBaselineTimestamp:
                        peaks.translation.baselineTimestamp,
                    rotationWindowBaselineTimestamp:
                        peaks.rotation.baselineTimestamp,
                    scaleWindowBaselineTimestamp: peaks.scale.baselineTimestamp,
                    normalizedStepTranslation: step.normalizedTranslation,
                    normalizedWindowTranslation:
                        peaks.translation.fit.normalizedTranslation,
                    stepRotationRadians: step.rotationRadians,
                    windowRotationRadians: peaks.rotation.fit.rotationRadians,
                    stepScaleFraction: step.scaleFraction,
                    windowScaleFraction: peaks.scale.fit.scaleFraction,
                    registrationQuality: quality,
                    backgroundFraction: background
                ))
            } else {
                anchors.append(current)
            }

            if let newestAnchor = anchors.last,
               current.timestamp - newestAnchor.timestamp >= anchorStride - 1e-9 {
                anchors.append(current)
            }
            previous = current
            try Task.checkCancellation()
        }

        try Task.checkCancellation()
        let terminalStatus: CameraMotionReaderTerminalStatus
        switch reader.status {
        case .completed:
            terminalStatus = .completed
        case .failed:
            terminalStatus = .failed
        case .cancelled:
            terminalStatus = .cancelled
        case .reading:
            terminalStatus = .reading
        case .unknown:
            terminalStatus = .unknown
        @unknown default:
            terminalStatus = .unknown
        }
        if let reason = CameraMotionReaderStatusPolicy.indeterminateReason(
            terminalStatus
        ) {
            throw AnalysisError.indeterminate(reason)
        }
        let nativeIntervals = zip(
            nativeSampleTimestamps.dropFirst(),
            nativeSampleTimestamps
        ).map { $0.0 - $0.1 }
        guard let cadence = CameraMotionCadencePolicy.assess(
            nativeFrameIntervals: nativeIntervals,
            nativeFrameDurations: nativeSampleDurations,
            derivativeWindowSeconds: derivativeWindowSeconds,
            hardMaximumGapSeconds: configuration.maximumNativeFrameGapSeconds,
            gapMultiplier: configuration.nativeFrameGapMultiplier,
            jitterAllowanceSeconds:
                configuration.nativeFrameJitterAllowanceSeconds
        ) else {
            throw AnalysisError.indeterminate(.timestampDiscontinuity)
        }
        if let reason = CameraMotionStreamCoveragePolicy.indeterminateReason(
            readerCompleted: true,
            requestedRange: range,
            firstSampleTimestamp: firstSampleTimestamp,
            lastSampleTimestamp: lastSampleTimestamp,
            lastSampleDuration: lastSampleDuration,
            maximumEndpointGapSeconds: cadence.allowedNativeFrameGapSeconds
        ) {
            throw AnalysisError.indeterminate(reason)
        }

        let reducerConfiguration = CameraMotionReducer.Configuration(
            derivativeWindowSeconds: derivativeWindowSeconds,
            allowedNativeFrameGapSeconds:
                cadence.allowedNativeFrameGapSeconds,
            robustNativeFrameIntervalSeconds:
                cadence.robustNativeFrameIntervalSeconds,
            staticMaximumNormalizedTranslation:
                configuration.provisionalStaticTranslation,
            movingMinimumNormalizedTranslation:
                configuration.provisionalMovingTranslation,
            staticMaximumRotationRadians:
                configuration.provisionalStaticRotationRadians,
            movingMinimumRotationRadians:
                configuration.provisionalMovingRotationRadians,
            staticMaximumScaleFraction:
                configuration.provisionalStaticScaleFraction,
            movingMinimumScaleFraction:
                configuration.provisionalMovingScaleFraction,
            minimumRegistrationQuality:
                configuration.minimumRegistrationQuality,
            minimumRegistrationPeakCorrelation:
                configuration.minimumRegistrationPeakCorrelation,
            minimumBackgroundFraction:
                configuration.minimumBackgroundFraction,
            analysisFrameSizePixels: analysisFrameSize,
            maximumAnalysisPixelCount: configuration.maximumAnalysisPixelCount,
            maximumAnalysisDimensionPixels:
                configuration.maximumAnalysisDimensionPixels,
            maximumAnalysisAspectRatio:
                configuration.maximumAnalysisAspectRatio,
            maximumRetainedPixelBufferBytes:
                configuration.maximumRetainedPixelBufferBytes,
            appearanceBoxAverageSizePixels:
                configuration.appearanceBoxAverageSizePixels,
            appearanceBoxAverageSpacingPixels:
                configuration.appearanceBoxAverageSpacingPixels,
            registrationCorrelationSampleSpacingPixels:
                configuration.registrationCorrelationSampleSpacingPixels,
            registrationPeakSearchRadiusPixels:
                configuration.registrationPeakSearchRadiusPixels,
            registrationPeakMinimumSeparationPixels:
                configuration.registrationPeakMinimumSeparationPixels,
            calibrationFingerprint: configuration.calibrationFingerprint
        )
        return CameraMotionReducer.reduce(
            observations: observations,
            configuration: reducerConfiguration
        )
    }

    private func estimate(
        reference: Snapshot,
        target: Snapshot
    ) throws -> CameraTileMotionFit {
        try reference.pixelBuffer.withUnsafeBuffer { referenceBuffer in
            try target.pixelBuffer.withUnsafeBuffer { targetBuffer in
                try Task.checkCancellation()
                guard CVPixelBufferGetWidth(referenceBuffer)
                        == CVPixelBufferGetWidth(targetBuffer),
                      CVPixelBufferGetHeight(referenceBuffer)
                        == CVPixelBufferGetHeight(targetBuffer),
                      CVPixelBufferGetPixelFormatType(referenceBuffer)
                        == kCVPixelFormatType_32BGRA,
                      CVPixelBufferGetPixelFormatType(targetBuffer)
                        == kCVPixelFormatType_32BGRA,
                      CameraAnalysisBufferBudget.isSupported(
                          widthPixels: CVPixelBufferGetWidth(referenceBuffer),
                          heightPixels: CVPixelBufferGetHeight(referenceBuffer),
                          bytesPerRow:
                              CVPixelBufferGetBytesPerRow(referenceBuffer),
                          maximumPixelCount:
                              configuration.maximumAnalysisPixelCount,
                          maximumDimensionPixels:
                              configuration.maximumAnalysisDimensionPixels,
                          maximumAspectRatio:
                              configuration.maximumAnalysisAspectRatio,
                          maximumRetainedBytes:
                              configuration.maximumRetainedPixelBufferBytes
                      ),
                      CameraAnalysisBufferBudget.isSupported(
                          widthPixels: CVPixelBufferGetWidth(targetBuffer),
                          heightPixels: CVPixelBufferGetHeight(targetBuffer),
                          bytesPerRow: CVPixelBufferGetBytesPerRow(targetBuffer),
                          maximumPixelCount:
                              configuration.maximumAnalysisPixelCount,
                          maximumDimensionPixels:
                              configuration.maximumAnalysisDimensionPixels,
                          maximumAspectRatio:
                              configuration.maximumAnalysisAspectRatio,
                          maximumRetainedBytes:
                              configuration.maximumRetainedPixelBufferBytes
                      ) else {
                    throw AnalysisError.indeterminate(.analysisResolutionUnsupported)
                }

                let width = CVPixelBufferGetWidth(referenceBuffer)
                let height = CVPixelBufferGetHeight(referenceBuffer)
                let backgroundPlan = CameraBackgroundRegionPlanner.make(
                    personBoxes: reference.personBoxes + target.personBoxes,
                    inflationFraction: configuration.personBoxInflationFraction,
                    minimumTileAreaFraction: configuration.minimumTileAreaFraction
                )
                guard let analysisTilePlan =
                        CameraBackgroundAnalysisTilePlanner.make(
                            backgroundTiles: backgroundPlan.tiles,
                            imageSizePixels: SIMD2(width, height),
                            minimumTileAreaFraction:
                                Double(configuration.minimumTileAreaFraction),
                            boxAverageSizePixels:
                                configuration.appearanceBoxAverageSizePixels,
                            boxAverageSpacingPixels:
                                configuration.appearanceBoxAverageSpacingPixels,
                            correlationSampleSpacingPixels:
                                configuration
                                    .registrationCorrelationSampleSpacingPixels,
                            peakSearchRadiusPixels:
                                configuration.registrationPeakSearchRadiusPixels
                        ), analysisTilePlan.sampledBackgroundFraction
                            >= configuration.minimumBackgroundFraction,
                      analysisTilePlan.tiles.count
                        >= configuration.tileFit.minimumTileCount else {
                    throw AnalysisError.indeterminate(.insufficientBackground)
                }

                let requests = analysisTilePlan.tiles.map {
                    tile -> VNTranslationalImageRegistrationRequest in
                    let request = VNTranslationalImageRegistrationRequest(
                        targetedCVPixelBuffer: targetBuffer,
                        orientation: .up,
                        options: [:]
                    )
                    request.revision = Self.translationalRegistrationRequestRevision
                    request.regionOfInterest = tile
                    return request
                }
                let handler = VNImageRequestHandler(
                    cvPixelBuffer: referenceBuffer,
                    orientation: .up,
                    options: [:]
                )
                do {
                    try handler.perform(requests)
                    try Task.checkCancellation()
                } catch is CancellationError {
                    throw CancellationError()
                } catch {
                    throw AnalysisError.indeterminate(.registrationFailed)
                }

                let diagonal = hypot(Double(width), Double(height))
                var registrations: [CameraTileRegistration] = []
                registrations.reserveCapacity(requests.count)
                for (tile, request) in zip(analysisTilePlan.tiles, requests) {
                    guard let transform = request.results?.first?.alignmentTransform,
                          let translation = CameraVisionTranslationPolicy.validate(
                              transform,
                              imageSizePixels: SIMD2(width, height)
                          ),
                          let appearance = Self.appearanceMetrics(
                              reference: referenceBuffer,
                              target: targetBuffer,
                              tile: tile,
                              roundedPixelShift:
                                  translation.roundedPixelShift,
                              boxAverageSizePixels:
                                  configuration.appearanceBoxAverageSizePixels,
                              boxAverageSpacingPixels:
                                  configuration.appearanceBoxAverageSpacingPixels,
                              correlationSampleSpacingPixels:
                                  configuration
                                    .registrationCorrelationSampleSpacingPixels,
                              peakSearchRadiusPixels:
                                  configuration.registrationPeakSearchRadiusPixels,
                              peakMinimumSeparationPixels:
                                  configuration
                                    .registrationPeakMinimumSeparationPixels,
                              minimumPeakCorrelation:
                                  configuration
                                    .minimumRegistrationPeakCorrelation
                          ) else {
                        throw AnalysisError.indeterminate(.registrationFailed)
                    }
                    registrations.append(CameraTileRegistration(
                        centrePixels: SIMD2(
                            (Double(tile.midX) - 0.5) * Double(width),
                            (Double(tile.midY) - 0.5) * Double(height)
                        ),
                        translationPixels: translation.translationPixels,
                        areaFraction: appearance.usableAreaFraction,
                        minimumStructureTensorEigenvalue:
                            appearance.minimumStructureTensorEigenvalue,
                        registrationUniqueness: appearance.registrationUniqueness,
                        postWarpMeanAbsoluteError:
                            appearance.postWarpMeanAbsoluteError
                    ))
                }
                guard CameraPlannedTileEvidencePolicy.isComplete(
                    plannedTileCount: analysisTilePlan.tiles.count,
                    validatedRegistrationCount: registrations.count
                ) else {
                    throw AnalysisError.indeterminate(.registrationFailed)
                }
                let qualityRegistrations = registrations.filter {
                    CameraTileMotionFitter.individuallyUsable(
                        $0,
                        configuration: configuration.tileFit
                    )
                }
                guard qualityRegistrations.count
                        >= configuration.tileFit.minimumTileCount else {
                    throw AnalysisError.indeterminate(.lowRegistrationQuality)
                }
                let qualityArea = qualityRegistrations.reduce(0.0) {
                    $0 + $1.areaFraction
                }
                guard qualityArea >= configuration.minimumBackgroundFraction else {
                    throw AnalysisError.indeterminate(.insufficientBackground)
                }
                guard let fit = CameraTileMotionFitter.fit(
                    qualityRegistrations,
                    imageDiagonalPixels: diagonal,
                    configuration: configuration.tileFit
                ) else {
                    throw AnalysisError.indeterminate(.registrationFailed)
                }
                guard fit.usableBackgroundFraction
                        >= configuration.minimumBackgroundFraction else {
                    throw AnalysisError.indeterminate(.insufficientBackground)
                }
                return fit
            }
        }
    }

    private struct AppearanceMetrics {
        let usableAreaFraction: Double
        let minimumStructureTensorEigenvalue: Double
        let registrationUniqueness: Double
        let postWarpMeanAbsoluteError: Double
    }

    /// Bounded-cost texture and post-warp residual measurement. The exact same
    /// rectangular sample grid feeds both images' structure tensors,
    /// uniqueness evidence and their pairwise appearance residual.
    private static func appearanceMetrics(
        reference: CVPixelBuffer,
        target: CVPixelBuffer,
        tile: CGRect,
        roundedPixelShift: SIMD2<Int>,
        boxAverageSizePixels: Int,
        boxAverageSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        peakSearchRadiusPixels: Int,
        peakMinimumSeparationPixels: Int,
        minimumPeakCorrelation: Double
    ) -> AppearanceMetrics? {
        guard CVPixelBufferLockBaseAddress(reference, .readOnly) == kCVReturnSuccess else {
            return nil
        }
        defer { CVPixelBufferUnlockBaseAddress(reference, .readOnly) }
        guard CVPixelBufferLockBaseAddress(target, .readOnly) == kCVReturnSuccess else {
            return nil
        }
        defer { CVPixelBufferUnlockBaseAddress(target, .readOnly) }

        guard let referenceBase = CVPixelBufferGetBaseAddress(reference),
              let targetBase = CVPixelBufferGetBaseAddress(target) else { return nil }
        let width = CVPixelBufferGetWidth(reference)
        let height = CVPixelBufferGetHeight(reference)
        let referenceStride = CVPixelBufferGetBytesPerRow(reference)
        let targetStride = CVPixelBufferGetBytesPerRow(target)

        guard let plan = CameraFixedBoxAverageGridPolicy.make(
            imageSizePixels: SIMD2(width, height),
            normalizedTile: tile,
            roundedPixelShift: roundedPixelShift,
            boxSizePixels: boxAverageSizePixels,
            boxSpacingPixels: boxAverageSpacingPixels
        ), let pair = CameraFixedBoxAveragePairSampler.sample(
            plan: plan,
            referenceLumaAt: { x, y in
                let row = height - 1 - y
                let pixel = referenceBase.advanced(
                    by: row * referenceStride + x * 4
                ).assumingMemoryBound(to: UInt8.self)
                return Self.luma(pixel)
            },
            targetLumaAt: { x, y in
                let row = height - 1 - y
                let pixel = targetBase.advanced(
                    by: row * targetStride + x * 4
                ).assumingMemoryBound(to: UInt8.self)
                return Self.luma(pixel)
            }
        ) else { return nil }
        let absoluteError = zip(
            pair.referenceBoxAverages,
            pair.targetBoxAverages
        ).reduce(0.0) { $0 + abs($1.0 - $1.1) }
        guard let referenceTexture = CameraTextureQualityAnalyzer.analyze(
            luma: pair.referenceBoxAverages,
            width: pair.width,
            height: pair.height
        ), let targetTexture = CameraTextureQualityAnalyzer.analyze(
            luma: pair.targetBoxAverages,
            width: pair.width,
            height: pair.height
        ), let peakEvidence = CameraRegistrationPeakAnalyzer.analyze(
            referenceBoxAverages: pair.referenceBoxAverages,
            targetBoxAverages: pair.targetBoxAverages,
            width: pair.width,
            height: pair.height,
            renderOriginPixels: pair.renderOriginPixels,
            boxSpacingPixels: pair.boxSpacingPixels,
            correlationSampleSpacingPixels:
                correlationSampleSpacingPixels,
            maximumSearchRadiusPixels: peakSearchRadiusPixels,
            minimumPeakSeparationPixels: peakMinimumSeparationPixels,
            minimumBestCorrelation: minimumPeakCorrelation
        ) else { return nil }
        return AppearanceMetrics(
            usableAreaFraction:
                Double(pair.usablePixelArea)
                    / (Double(width) * Double(height)),
            minimumStructureTensorEigenvalue: min(
                referenceTexture.minimumStructureTensorEigenvalue,
                targetTexture.minimumStructureTensorEigenvalue
            ),
            registrationUniqueness:
                peakEvidence.normalizedPeakSeparation,
            postWarpMeanAbsoluteError:
                absoluteError / Double(pair.referenceBoxAverages.count)
        )
    }

    private static func luma(_ bgra: UnsafePointer<UInt8>) -> Double {
        // ITU-R BT.601 integer approximation over BGRA input.
        Double((29 * Int(bgra[0]) + 150 * Int(bgra[1])
                + 77 * Int(bgra[2])) >> 8)
    }

    private static func personBoxes(in pixelBuffer: CVPixelBuffer) throws -> [CGRect] {
        let request = VNDetectHumanRectanglesRequest()
        request.revision = Self.humanRectanglesRequestRevision
        request.upperBodyOnly = false
        let handler = VNImageRequestHandler(
            cvPixelBuffer: pixelBuffer,
            orientation: .up,
            options: [:]
        )
        try handler.perform([request])
        return (request.results ?? []).map(\.boundingBox).filter {
            $0.width > 0 && $0.height > 0
        }
    }
}
