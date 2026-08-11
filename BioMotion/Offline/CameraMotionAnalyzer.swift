import CoreGraphics
import Foundation
import simd

/// One native-frame camera-registration interval. The adapter supplies both a
/// one-frame displacement (to catch a jump across a rolling-window boundary)
/// and displacement from a short-lived anchor (so sub-pixel-per-frame drift
/// accumulates above Vision's integer-pixel registration resolution). All
/// translations are fractions of the upright image diagonal.
struct CameraMotionObservation: Equatable, Sendable {
    let startTimestamp: TimeInterval
    let endTimestamp: TimeInterval
    /// Each metric keeps its own peak anchor. Collapsing them to one composite
    /// "worst" pair can hide a moving-band crossing when calibrated band ratios
    /// differ between translation, rotation and scale.
    let translationWindowBaselineTimestamp: TimeInterval
    let rotationWindowBaselineTimestamp: TimeInterval
    let scaleWindowBaselineTimestamp: TimeInterval
    let normalizedStepTranslation: SIMD2<Double>
    let normalizedWindowTranslation: SIMD2<Double>
    /// In-plane image rotation fitted from multiple background tiles. A single
    /// global translation cannot distinguish this motion.
    let stepRotationRadians: Double
    let windowRotationRadians: Double
    /// Fractional isotropic image scale change fitted with the rotation. This
    /// catches zoom/push-in evidence that translation alone cannot observe.
    let stepScaleFraction: Double
    let windowScaleFraction: Double
    /// Adapter-derived quality (texture, post-warp residual and tile fit), not
    /// `VNObservation.confidence`, which Vision documents may always be 1.
    let registrationQuality: Double
    let backgroundFraction: Double
}

/// Auditable numbers behind a camera-reference decision. The translation value
/// is the largest native-step or rolling-anchor excursion. It is deliberately
/// not a sum of adjacent registration magnitudes: that would turn stationary
/// pixel noise into motion in direct proportion to capture frame rate.
struct CameraMotionEvidence: Equatable, Sendable {
    let analyzedDurationSeconds: TimeInterval
    let derivativeWindowSeconds: TimeInterval
    let peakNormalizedTranslation: Double
    let translationStaticUpperBound: Double
    let translationMovingLowerBound: Double
    let peakRotationRadians: Double
    let rotationStaticUpperBoundRadians: Double
    let rotationMovingLowerBoundRadians: Double
    let peakScaleFraction: Double
    let scaleStaticUpperBound: Double
    let scaleMovingLowerBound: Double
    /// Nil means these numeric bands are only a pre-registered implementation
    /// proposal and have not passed the required camera-fixture calibration.
    let calibrationProfileID: String?
    /// Upright, post-composition dimensions actually presented to Vision. A
    /// calibration profile is bound to this exact render size; changing it also
    /// changes quantisation and residual statistics.
    let analysisFrameWidthPixels: Int
    let analysisFrameHeightPixels: Int
    let maximumAnalysisPixelCount: Int
    let measuredIntervals: Int
    let sampledFrames: Int
}

enum CameraReferenceIndeterminateReason: Equatable, Sendable {
    case insufficientMeasurements
    case insufficientCoverage
    case timestampDiscontinuity
    case lowRegistrationQuality
    case insufficientBackground
    case videoReadFailed
    case incompleteVideoRead
    case incompleteRangeCoverage
    case multipleVideoTracks
    case analysisResolutionUnsupported
    case analysisBudgetExceeded
    case personDetectionFailed
    case registrationFailed
    case invalidMeasurement
}

struct CameraMotionAnalysisRange: Equatable, Sendable {
    let startSeconds: TimeInterval
    let endSeconds: TimeInterval
}

/// Fixed compute admission for the native-frame adapter. The range bound keeps
/// sparse pose sampling from silently expanding into minutes of native Vision
/// work; the sample bound is an independent defence against malformed timing,
/// unexpectedly dense VFR media, or a misleading nominal frame rate.
enum CameraMotionScanBudgetPolicy {
    static let maximumRangeDurationSeconds: TimeInterval = 4.0
    static let maximumNativeSampleCount = 1_000

    static func rangeIndeterminateReason(
        _ range: CameraMotionAnalysisRange
    ) -> CameraReferenceIndeterminateReason? {
        guard range.startSeconds.isFinite,
              range.endSeconds.isFinite,
              range.startSeconds >= 0,
              range.endSeconds > range.startSeconds else {
            return .invalidMeasurement
        }
        let duration = range.endSeconds - range.startSeconds
        guard duration.isFinite else { return .invalidMeasurement }
        return duration <= maximumRangeDurationSeconds + 1e-9
            ? nil
            : .analysisBudgetExceeded
    }

    static func sampleIndeterminateReason(
        nativeSampleCount: Int
    ) -> CameraReferenceIndeterminateReason? {
        guard nativeSampleCount >= 0 else { return .invalidMeasurement }
        return nativeSampleCount <= maximumNativeSampleCount
            ? nil
            : .analysisBudgetExceeded
    }
}

struct CameraMotionCadenceAssessment: Equatable, Sendable {
    let robustNativeFrameIntervalSeconds: TimeInterval
    let allowedNativeFrameGapSeconds: TimeInterval
}

/// Derives one per-clip gap allowance from actual native PTS. Nominal frame-rate
/// metadata never enters this policy. A robust median keeps one dropped frame
/// from redefining the baseline, while every observed interval must still fit
/// the derived allowance.
enum CameraMotionCadencePolicy {
    static func assess(
        nativeFrameIntervals: [TimeInterval],
        nativeFrameDurations: [TimeInterval]? = nil,
        derivativeWindowSeconds: TimeInterval,
        hardMaximumGapSeconds: TimeInterval,
        gapMultiplier: Double,
        jitterAllowanceSeconds: TimeInterval
    ) -> CameraMotionCadenceAssessment? {
        guard !nativeFrameIntervals.isEmpty,
              nativeFrameIntervals.allSatisfy({ $0.isFinite && $0 > 0 }),
              nativeFrameDurations.map({
                  $0.count == nativeFrameIntervals.count + 1
                      && $0.allSatisfy { $0.isFinite && $0 > 0 }
              }) ?? true,
              derivativeWindowSeconds.isFinite,
              derivativeWindowSeconds > 0,
              hardMaximumGapSeconds.isFinite,
              hardMaximumGapSeconds > 0,
              gapMultiplier.isFinite,
              gapMultiplier >= 1,
              jitterAllowanceSeconds.isFinite,
              jitterAllowanceSeconds >= 0 else { return nil }

        let sorted = nativeFrameIntervals.sorted()
        let middle = sorted.count / 2
        let robustInterval: TimeInterval
        if sorted.count.isMultiple(of: 2) {
            robustInterval = (sorted[middle - 1] + sorted[middle]) / 2
        } else {
            robustInterval = sorted[middle]
        }
        guard robustInterval.isFinite, robustInterval > 0 else { return nil }

        // One SG derivative window contains exactly windowSize - 1 native
        // intervals. This catches a stream whose metadata says 240 fps while its
        // actual PTS are 30 fps; a long sparse-pose window still admits a truly
        // denser native stream.
        let targetInterval = derivativeWindowSeconds
            / Double(SavitzkyGolayFilter.windowSize - 1)
        guard targetInterval.isFinite, targetInterval > 0,
              robustInterval <= targetInterval + jitterAllowanceSeconds + 1e-12
        else { return nil }

        let allowedGap = min(
            hardMaximumGapSeconds,
            robustInterval * gapMultiplier + jitterAllowanceSeconds
        )
        guard allowedGap.isFinite,
              allowedGap > 0,
              allowedGap < derivativeWindowSeconds,
              nativeFrameIntervals.allSatisfy({
                  $0 <= allowedGap + 1e-12
              }),
              (nativeFrameDurations?.allSatisfy({
                  $0 <= allowedGap + 1e-12
              }) ?? true) else { return nil }
        return CameraMotionCadenceAssessment(
            robustNativeFrameIntervalSeconds: robustInterval,
            allowedNativeFrameGapSeconds: allowedGap
        )
    }
}

/// Strict source-layout policy. The AVVideoComposition convenience constructor
/// only preserves original source timing automatically for exactly one video
/// track; multi-track assets are therefore rejected until the product owns an
/// explicit primary-track selection policy.
enum CameraMotionVideoTrackPolicy {
    static func indeterminateReason(
        videoTrackCount: Int
    ) -> CameraReferenceIndeterminateReason? {
        guard videoTrackCount == 1 else {
            return videoTrackCount > 1 ? .multipleVideoTracks : .videoReadFailed
        }
        return nil
    }
}

enum CameraMotionReaderTerminalStatus: Equatable, Sendable {
    case completed
    case failed
    case cancelled
    case reading
    case unknown
}

/// Keeps I/O failure distinct from a clean-but-incomplete range. Cancellation
/// from the Swift task is checked before this policy; a reader that independently
/// reports cancellation is therefore an incomplete read, not task cancellation.
enum CameraMotionReaderStatusPolicy {
    static func indeterminateReason(
        _ status: CameraMotionReaderTerminalStatus
    ) -> CameraReferenceIndeterminateReason? {
        switch status {
        case .completed:
            return nil
        case .failed:
            return .videoReadFailed
        case .cancelled, .reading, .unknown:
            return .incompleteVideoRead
        }
    }
}

/// Pure end-to-end PTS coverage check. Measuring one valid derivative window is
/// not enough to license dynamics for a longer requested range.
enum CameraMotionStreamCoveragePolicy {
    static func indeterminateReason(
        readerCompleted: Bool,
        requestedRange: CameraMotionAnalysisRange,
        firstSampleTimestamp: TimeInterval?,
        lastSampleTimestamp: TimeInterval?,
        lastSampleDuration: TimeInterval?,
        maximumEndpointGapSeconds: TimeInterval
    ) -> CameraReferenceIndeterminateReason? {
        guard readerCompleted else { return .incompleteVideoRead }
        guard requestedRange.startSeconds.isFinite,
              requestedRange.endSeconds.isFinite,
              requestedRange.endSeconds > requestedRange.startSeconds,
              maximumEndpointGapSeconds.isFinite,
              maximumEndpointGapSeconds >= 0,
              let firstSampleTimestamp,
              let lastSampleTimestamp,
              firstSampleTimestamp.isFinite,
              lastSampleTimestamp.isFinite,
              lastSampleTimestamp >= firstSampleTimestamp else {
            return .incompleteRangeCoverage
        }

        guard let duration = lastSampleDuration,
              duration.isFinite,
              duration > 0 else { return .incompleteRangeCoverage }
        let tolerance = 1e-9
        let coveredEnd = lastSampleTimestamp + duration
        guard coveredEnd.isFinite else { return .incompleteRangeCoverage }
        // Coverage is directional. A sample before the requested start or one
        // ending after the requested end covers the boundary; absolute distance
        // would incorrectly reject that strictly stronger evidence.
        let coversStart = firstSampleTimestamp
            <= requestedRange.startSeconds + maximumEndpointGapSeconds + tolerance
        let coversEnd = coveredEnd
            >= requestedRange.endSeconds - maximumEndpointGapSeconds - tolerance
        return coversStart && coversEnd ? nil : .incompleteRangeCoverage
    }
}

/// Bounds the number of simultaneously retained BGRA pixels while preserving
/// the upright aspect ratio. Width/height are kept even for common video codecs.
enum CameraAnalysisRenderBudget {
    static func scaledPixelDimensions(
        uprightSize: CGSize,
        maximumPixelCount: Int,
        maximumDimensionPixels: Int,
        maximumAspectRatio: Double
    ) -> SIMD2<Int>? {
        guard uprightSize.width.isFinite, uprightSize.height.isFinite,
              uprightSize.width > 0, uprightSize.height > 0,
              maximumPixelCount >= 4,
              maximumDimensionPixels >= 2,
              maximumAspectRatio.isFinite,
              maximumAspectRatio >= 1 else { return nil }
        let area = uprightSize.width * uprightSize.height
        guard area.isFinite, area > 0 else { return nil }
        let aspectRatio = max(uprightSize.width, uprightSize.height)
            / min(uprightSize.width, uprightSize.height)
        guard aspectRatio.isFinite,
              aspectRatio <= maximumAspectRatio else { return nil }
        let scale = min(
            1,
            sqrt(Double(maximumPixelCount) / Double(area)),
            Double(maximumDimensionPixels) / Double(uprightSize.width),
            Double(maximumDimensionPixels) / Double(uprightSize.height)
        )
        let halfWidth = floor(uprightSize.width * scale / 2)
        let halfHeight = floor(uprightSize.height * scale / 2)
        guard halfWidth >= 1, halfHeight >= 1,
              halfWidth < Double(Int.max / 2),
              halfHeight < Double(Int.max / 2) else { return nil }
        let width = Int(halfWidth) * 2
        let height = Int(halfHeight) * 2
        guard CameraAnalysisBufferBudget.dimensionsAreSupported(
            widthPixels: width,
            heightPixels: height,
            maximumPixelCount: maximumPixelCount,
            maximumDimensionPixels: maximumDimensionPixels,
            maximumAspectRatio: maximumAspectRatio
        ) else { return nil }
        return SIMD2(width, height)
    }

    static func acceptedPixelDimensions(
        renderSize: CGSize,
        maximumPixelCount: Int,
        maximumDimensionPixels: Int,
        maximumAspectRatio: Double
    ) -> SIMD2<Int>? {
        guard renderSize.width.isFinite, renderSize.height.isFinite,
              renderSize.width > 0, renderSize.height > 0,
              renderSize.width.rounded() == renderSize.width,
              renderSize.height.rounded() == renderSize.height,
              renderSize.width < Double(Int.max),
              renderSize.height < Double(Int.max) else { return nil }
        let width = Int(renderSize.width)
        let height = Int(renderSize.height)
        guard CameraAnalysisBufferBudget.dimensionsAreSupported(
            widthPixels: width,
            heightPixels: height,
            maximumPixelCount: maximumPixelCount,
            maximumDimensionPixels: maximumDimensionPixels,
            maximumAspectRatio: maximumAspectRatio
        ) else { return nil }
        return SIMD2(width, height)
    }
}

/// The app can retain the current/previous frame and up to three window
/// anchors. Budgeting those five real pixel buffers (including row padding)
/// makes the memory owned by this adapter explicit; framework pools are not
/// counted or controlled here.
enum CameraAnalysisBufferBudget {
    static let maximumRetainedBufferCount = 5

    static func dimensionsAreSupported(
        widthPixels: Int,
        heightPixels: Int,
        maximumPixelCount: Int,
        maximumDimensionPixels: Int,
        maximumAspectRatio: Double
    ) -> Bool {
        guard widthPixels >= 2, heightPixels >= 2,
              widthPixels.isMultiple(of: 2),
              heightPixels.isMultiple(of: 2),
              maximumPixelCount >= 4,
              maximumDimensionPixels >= 2,
              widthPixels <= maximumDimensionPixels,
              heightPixels <= maximumDimensionPixels,
              maximumAspectRatio.isFinite,
              maximumAspectRatio >= 1,
              widthPixels <= maximumPixelCount / heightPixels else {
            return false
        }
        let shorter = min(widthPixels, heightPixels)
        let longer = max(widthPixels, heightPixels)
        return Double(longer) / Double(shorter) <= maximumAspectRatio
    }

    static func isSupported(
        widthPixels: Int,
        heightPixels: Int,
        bytesPerRow: Int,
        maximumPixelCount: Int,
        maximumDimensionPixels: Int,
        maximumAspectRatio: Double,
        maximumRetainedBytes: Int
    ) -> Bool {
        guard dimensionsAreSupported(
            widthPixels: widthPixels,
            heightPixels: heightPixels,
            maximumPixelCount: maximumPixelCount,
            maximumDimensionPixels: maximumDimensionPixels,
            maximumAspectRatio: maximumAspectRatio
        ), maximumRetainedBytes > 0,
           widthPixels <= Int.max / 4 else { return false }
        let minimumBytesPerRow = widthPixels * 4
        guard bytesPerRow >= minimumBytesPerRow,
              bytesPerRow <= maximumRetainedBytes / heightPixels else {
            return false
        }
        let bytesPerBuffer = bytesPerRow * heightPixels
        return bytesPerBuffer
            <= maximumRetainedBytes / maximumRetainedBufferCount
    }
}

struct CameraVideoFormatGeometry: Equatable, Sendable {
    let encodedDimensions: SIMD2<Int>
    let cleanAperture: CGRect
    let pixelAspectAdjustedDimensions: CGSize
}

/// Calibration operates in encoded render pixels. Cropped clean apertures,
/// anamorphic pixels, or mid-track raster changes would make those pixels mean
/// something different, so the adapter admits only one exact full-raster domain.
enum CameraVideoFormatGeometryPolicy {
    static func isSupported(
        naturalSize: CGSize,
        formatGeometries: [CameraVideoFormatGeometry]
    ) -> Bool {
        guard !formatGeometries.isEmpty,
              naturalSize.width.isFinite, naturalSize.height.isFinite,
              naturalSize.width >= 2, naturalSize.height >= 2,
              naturalSize.width.rounded() == naturalSize.width,
              naturalSize.height.rounded() == naturalSize.height,
              let naturalWidth = Int(exactly: Double(naturalSize.width)),
              let naturalHeight = Int(exactly: Double(naturalSize.height)) else {
            return false
        }
        let expectedDimensions = SIMD2(naturalWidth, naturalHeight)
        let expectedAperture = CGRect(
            x: 0,
            y: 0,
            width: naturalWidth,
            height: naturalHeight
        )
        return formatGeometries.allSatisfy { format in
            format.encodedDimensions == expectedDimensions
                && format.cleanAperture.origin.x.isFinite
                && format.cleanAperture.origin.y.isFinite
                && format.cleanAperture.width.isFinite
                && format.cleanAperture.height.isFinite
                && format.cleanAperture == expectedAperture
                && format.pixelAspectAdjustedDimensions.width.isFinite
                && format.pixelAspectAdjustedDimensions.height.isFinite
                && format.pixelAspectAdjustedDimensions == naturalSize
        }
    }
}

/// Fail-closed identity check used before the adapter is admitted. A profile ID
/// without an exact bounded render domain is metadata, not calibration.
enum CameraCalibrationReadinessPolicy {
    static func isReady(
        profileID: String?,
        frameSizePixels: SIMD2<Int>?,
        maximumAnalysisPixelCount: Int,
        maximumAnalysisDimensionPixels: Int,
        maximumAnalysisAspectRatio: Double,
        maximumRetainedPixelBufferBytes: Int,
        appearanceBoxAverageSizePixels: Int,
        appearanceBoxAverageSpacingPixels: Int,
        registrationCorrelationSampleSpacingPixels: Int,
        registrationPeakSearchRadiusPixels: Int,
        registrationPeakMinimumSeparationPixels: Int,
        minimumTileAreaFraction: Double,
        derivativeWindowDomainSeconds: ClosedRange<TimeInterval>?,
        nativeFrameIntervalDomainSeconds: ClosedRange<TimeInterval>?,
        claimedFingerprintMatchesConfiguration: Bool
    ) -> Bool {
        guard let profileID,
              !profileID.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty,
              let frameSizePixels,
              frameSizePixels.x >= 2,
              frameSizePixels.y >= 2,
              frameSizePixels.x.isMultiple(of: 2),
              frameSizePixels.y.isMultiple(of: 2),
              frameSizePixels.x <= Int.max / 4,
              CameraAnalysisBufferBudget.isSupported(
                  widthPixels: frameSizePixels.x,
                  heightPixels: frameSizePixels.y,
                  bytesPerRow: frameSizePixels.x * 4,
                  maximumPixelCount: maximumAnalysisPixelCount,
                  maximumDimensionPixels: maximumAnalysisDimensionPixels,
                  maximumAspectRatio: maximumAnalysisAspectRatio,
                  maximumRetainedBytes: maximumRetainedPixelBufferBytes
              ),
              CameraCalibrationAppearanceGeometryPolicy.isSupported(
                  frameSizePixels: frameSizePixels,
                  boxAverageSizePixels: appearanceBoxAverageSizePixels,
                  boxAverageSpacingPixels: appearanceBoxAverageSpacingPixels,
                  correlationSampleSpacingPixels:
                      registrationCorrelationSampleSpacingPixels,
                  peakSearchRadiusPixels:
                      registrationPeakSearchRadiusPixels,
                  peakMinimumSeparationPixels:
                      registrationPeakMinimumSeparationPixels,
                  minimumTileAreaFraction: minimumTileAreaFraction
              ),
              valid(domain: derivativeWindowDomainSeconds),
              valid(domain: nativeFrameIntervalDomainSeconds),
              claimedFingerprintMatchesConfiguration else {
            return false
        }
        return true
    }

    private static func valid(
        domain: ClosedRange<TimeInterval>?
    ) -> Bool {
        guard let domain,
              domain.lowerBound.isFinite,
              domain.upperBound.isFinite,
              domain.lowerBound > 0,
              domain.upperBound >= domain.lowerBound else { return false }
        return true
    }
}

struct CameraAnalysisRenderGeometry: Equatable, Sendable {
    let pixelDimensions: SIMD2<Int>
    let sourceToRenderTransform: CGAffineTransform
}

/// Pure preferred-transform normalization used by the AVFoundation adapter.
/// The transformed source bounds are checked against the render canvas here so
/// an order mistake in Core Graphics concatenation fails closed instead of
/// silently cropping a rotated or mirrored source.
enum CameraAnalysisRenderGeometryPolicy {
    static func make(
        naturalSize: CGSize,
        preferredTransform: CGAffineTransform,
        maximumPixelCount: Int,
        maximumDimensionPixels: Int,
        maximumAspectRatio: Double
    ) -> CameraAnalysisRenderGeometry? {
        guard naturalSize.width.isFinite, naturalSize.height.isFinite,
              naturalSize.width > 0, naturalSize.height > 0,
              preferredTransform.a.isFinite,
              preferredTransform.b.isFinite,
              preferredTransform.c.isFinite,
              preferredTransform.d.isFinite,
              preferredTransform.tx.isFinite,
              preferredTransform.ty.isFinite else { return nil }
        let a = Double(preferredTransform.a)
        let b = Double(preferredTransform.b)
        let c = Double(preferredTransform.c)
        let d = Double(preferredTransform.d)
        let firstColumnLength = hypot(a, b)
        let secondColumnLength = hypot(c, d)
        let columnDotProduct = a * c + b * d
        let determinant = a * d - b * c
        let linearTransformTolerance = 1e-4
        guard abs(firstColumnLength - 1) <= linearTransformTolerance,
              abs(secondColumnLength - 1) <= linearTransformTolerance,
              abs(columnDotProduct) <= linearTransformTolerance,
              abs(abs(determinant) - 1) <= linearTransformTolerance else {
            return nil
        }
        let sourceBounds = CGRect(origin: .zero, size: naturalSize)
        let uprightBounds = sourceBounds.applying(preferredTransform).standardized
        guard let dimensions = CameraAnalysisRenderBudget.scaledPixelDimensions(
            uprightSize: uprightBounds.size,
            maximumPixelCount: maximumPixelCount,
            maximumDimensionPixels: maximumDimensionPixels,
            maximumAspectRatio: maximumAspectRatio
        ) else { return nil }
        let scale = min(
            CGFloat(dimensions.x) / uprightBounds.width,
            CGFloat(dimensions.y) / uprightBounds.height
        )
        guard scale.isFinite, scale > 0 else { return nil }
        let transform = preferredTransform
            .concatenating(CGAffineTransform(
                translationX: -uprightBounds.minX,
                y: -uprightBounds.minY
            ))
            .concatenating(CGAffineTransform(scaleX: scale, y: scale))
        let transformedBounds = sourceBounds.applying(transform).standardized
        let tolerance: CGFloat = 1e-6
        guard transformedBounds.minX >= -tolerance,
              transformedBounds.minY >= -tolerance,
              transformedBounds.maxX <= CGFloat(dimensions.x) + tolerance,
              transformedBounds.maxY <= CGFloat(dimensions.y) + tolerance else {
            return nil
        }
        return CameraAnalysisRenderGeometry(
            pixelDimensions: dimensions,
            sourceToRenderTransform: transform
        )
    }
}

protocol CameraMotionAnalyzing: Sendable {
    /// True only when this exact adapter configuration owns a non-empty,
    /// versioned calibration identity and a bounded frame-size domain. Protocol
    /// conformers fail closed unless they opt in explicitly.
    var isVersionedCalibrationReady: Bool { get }

    func analyzeVideo(
        at url: URL,
        range: CameraMotionAnalysisRange,
        derivativeWindowSeconds: TimeInterval
    ) async throws -> CameraReferenceState
}

extension CameraMotionAnalyzing {
    var isVersionedCalibrationReady: Bool { false }
}

/// Pure runner admission. The higher-priority contact capability is resolved
/// before calibration so the current bundled models never pay for evidence that
/// cannot change their product output. Unknown capability is unavailable, not a
/// reason to speculate that a future model might pass.
enum CameraReferenceAnalysisAdmission: Equatable, Sendable {
    case singleFrame
    case contactUnavailable
    case calibrationUnavailable
    case analyze

    static func decide(
        isSingleFrame: Bool,
        hasValidatedFootContactSupport: Bool?,
        isVersionedCalibrationReady: Bool
    ) -> Self {
        if isSingleFrame { return .singleFrame }
        guard hasValidatedFootContactSupport == true else {
            return .contactUnavailable
        }
        guard isVersionedCalibrationReady else {
            return .calibrationUnavailable
        }
        return .analyze
    }

    /// State a runner can finalize without opening the video adapter. `.analyze`
    /// deliberately has no optimistic placeholder; authorization remains denied
    /// until the adapter returns a measured state.
    var resolvedState: CameraReferenceState? {
        switch self {
        case .singleFrame: return .notRequiredForSingleFrame
        case .contactUnavailable: return .unmeasured
        case .calibrationUnavailable: return .calibrationUnavailable
        case .analyze: return nil
        }
    }
}

/// Pure orchestration arithmetic shared by the runner and tests. Nominal frame
/// rate is used only to choose the requested source interval and the pose
/// derivative window; the analyzer still enumerates actual native sample PTS.
enum CameraAnalysisPolicy {
    static func derivativeWindowSeconds(
        samplingMode: FrameSource.SamplingMode,
        nominalFrameRate: Double
    ) -> TimeInterval? {
        let rate: Double
        switch samplingMode {
        case .singleFrame:
            return nil
        case .fps(let requested):
            rate = FrameSource.sanitisedFrameRate(requested)
        case .nativeWindow:
            rate = FrameSource.sanitisedFrameRate(nominalFrameRate)
        }
        return Double(SavitzkyGolayFilter.windowSize - 1) / rate
    }

    static func analysisRange(
        requestedTimestamps: [TimeInterval],
        assetDuration: TimeInterval,
        nominalFrameRate: Double
    ) -> CameraMotionAnalysisRange? {
        guard assetDuration.isFinite, assetDuration > 0,
              let first = requestedTimestamps.first,
              let last = requestedTimestamps.last,
              first.isFinite, last.isFinite, first >= 0, last >= first else {
            return nil
        }
        // AVAssetReader's range is half-open. Extend by one nominal interval to
        // include the source sample at the last requested timestamp; this value
        // bounds the read only and is never used as a measurement timestamp.
        let interval = 1 / FrameSource.sanitisedFrameRate(nominalFrameRate)
        let end = min(assetDuration, last + interval)
        guard end > first else { return nil }
        return CameraMotionAnalysisRange(startSeconds: first, endSeconds: end)
    }
}

/// Whether the image sequence supplies a stable camera reference for a
/// derivative-based dynamics window. This is deliberately separate from
/// `NimbleEngine.MotionVerdict`, which describes subject motion.
enum CameraReferenceState: Equatable, Sendable {
    /// No source policy or measurement has been established for this session.
    /// This is the fail-closed store default, not shorthand for zero motion.
    case unmeasured
    /// A single source image has no temporal camera path. It may support an
    /// explicitly static-equilibrium solve, but never temporal/gait dynamics.
    case notRequiredForSingleFrame
    case staticWithinBudget(CameraMotionEvidence)
    case moving(CameraMotionEvidence)
    /// Valid measurements landed between the calibration profile's static and
    /// moving bands. The product must not force a binary answer in this zone.
    case betweenCalibrationBands(CameraMotionEvidence)
    /// Metrics were measured, but no versioned held-out calibration profile is
    /// installed. This is intentionally non-permissive.
    case calibrationRequired(CameraMotionEvidence)
    /// The adapter was not run because this build has no versioned calibration
    /// profile for a bounded analysis-frame domain. No numeric evidence exists,
    /// and re-filming cannot change this build capability.
    case calibrationUnavailable
    case indeterminate(CameraReferenceIndeterminateReason)

    var permitsStaticEquilibrium: Bool {
        switch self {
        case .notRequiredForSingleFrame, .staticWithinBudget: return true
        case .unmeasured, .moving, .betweenCalibrationBands,
             .calibrationRequired, .calibrationUnavailable,
             .indeterminate: return false
        }
    }

    var permitsTemporalDynamics: Bool {
        if case .staticWithinBudget = self { return true }
        return false
    }

    var bannerTitle: String? {
        switch self {
        case .notRequiredForSingleFrame: return nil
        case .unmeasured: return "Camera reference not checked"
        case .staticWithinBudget:
            return "Visible background within calibrated motion budget"
        case .moving:
            return "Camera/background motion above calibrated band"
        case .betweenCalibrationBands:
            return "Camera motion is between calibrated bands"
        case .calibrationRequired:
            return "Camera motion calibration required"
        case .calibrationUnavailable:
            return "Camera calibration is not installed"
        case .indeterminate:
            return "Camera reference could not be verified"
        }
    }

    var bannerDetail: String? {
        switch self {
        case .notRequiredForSingleFrame:
            return nil
        case .unmeasured:
            return "No native-frame background measurement is available. Pose and anatomy remain available; dynamics fail closed."
        case .staticWithinBudget(let evidence):
            return Self.metricSentence(evidence)
                + " Visible-background image motion is inside the versioned calibration profile. This does not prove absolute physical camera translation is zero."
        case .moving(let evidence):
            return Self.metricSentence(evidence)
                + " Pose, anatomy and contact timing remain available; dynamics are withheld."
        case .betweenCalibrationBands(let evidence):
            return Self.metricSentence(evidence)
                + " The result lies between the registered static and moving bands, so dynamics fail closed."
        case .calibrationRequired(let evidence):
            return Self.metricSentence(evidence)
                + " No versioned tripod/moving-control profile validates these provisional bands. Pose, anatomy and contact timing remain available; dynamics are withheld."
        case .calibrationUnavailable:
            return "This build has no versioned camera-motion calibration profile. Pose, anatomy and contact timing remain available; re-filming cannot enable dynamics."
        case .indeterminate(let reason):
            let cause: String
            switch reason {
            case .insufficientMeasurements: cause = "too few native frames"
            case .insufficientCoverage: cause = "the measured span is shorter than one derivative window"
            case .timestampDiscontinuity: cause = "the native media timestamps contain a gap"
            case .lowRegistrationQuality: cause = "background registration quality is too low"
            case .insufficientBackground: cause = "too little background remains outside the person"
            case .videoReadFailed: cause = "the native video stream could not be read"
            case .incompleteVideoRead: cause = "the native video reader did not complete"
            case .incompleteRangeCoverage: cause = "native timestamps do not cover the requested analysis range"
            case .multipleVideoTracks: cause = "the asset contains multiple video tracks with no approved primary-track policy"
            case .analysisResolutionUnsupported: cause = "the upright analysis frame exceeds the bounded pixel budget"
            case .analysisBudgetExceeded: cause = "the requested native-frame scan exceeds the fixed analysis budget"
            case .personDetectionFailed: cause = "the person could not be excluded from the background"
            case .registrationFailed: cause = "the background tiles could not produce a consistent transform"
            case .invalidMeasurement: cause = "the registration evidence is invalid"
            }
            return "Reason: \(cause). Pose and anatomy remain available; dynamics fail closed."
        }
    }

    private static func metricSentence(_ evidence: CameraMotionEvidence) -> String {
        String(
            format: "Peak over %.2f s: translation %.1f%% of image diagonal (static ≤ %.1f%%, moving ≥ %.1f%%), rotation %.2f° (static ≤ %.2f°, moving ≥ %.2f°), scale %.1f%% (static ≤ %.1f%%, moving ≥ %.1f%%).",
            evidence.derivativeWindowSeconds,
            evidence.peakNormalizedTranslation * 100,
            evidence.translationStaticUpperBound * 100,
            evidence.translationMovingLowerBound * 100,
            evidence.peakRotationRadians * 180 / .pi,
            evidence.rotationStaticUpperBoundRadians * 180 / .pi,
            evidence.rotationMovingLowerBoundRadians * 180 / .pi,
            evidence.peakScaleFraction * 100,
            evidence.scaleStaticUpperBound * 100,
            evidence.scaleMovingLowerBound * 100
        )
    }
}

// MARK: - Background geometry and multi-tile motion fit

struct CameraBackgroundRegionPlan: Equatable, Sendable {
    let excludedPersonRegion: CGRect
    let tiles: [CGRect]
    let backgroundFraction: Double
}

/// Splits the complement of the union of the two person boxes into disjoint
/// rectangles. Vision accepts one rectangular ROI per request, so four requests
/// represent the actual background without painting a synthetic mask edge into
/// the images (which can itself dominate registration).
enum CameraBackgroundRegionPlanner {
    static func make(
        personBoxes: [CGRect],
        inflationFraction: CGFloat,
        minimumTileAreaFraction: CGFloat
    ) -> CameraBackgroundRegionPlan {
        let unit = CGRect(x: 0, y: 0, width: 1, height: 1)
        guard inflationFraction.isFinite, inflationFraction >= 0,
              minimumTileAreaFraction.isFinite, minimumTileAreaFraction >= 0,
              !personBoxes.isEmpty else {
            return CameraBackgroundRegionPlan(
                excludedPersonRegion: .null,
                tiles: [],
                backgroundFraction: 0
            )
        }

        let clipped = personBoxes.map { $0.standardized.intersection(unit) }
            .filter { !$0.isNull && !$0.isEmpty }
        guard var excluded = clipped.first else {
            return CameraBackgroundRegionPlan(
                excludedPersonRegion: .null,
                tiles: [],
                backgroundFraction: 0
            )
        }
        for box in clipped.dropFirst() { excluded = excluded.union(box) }
        excluded = excluded.insetBy(dx: -inflationFraction, dy: -inflationFraction)
            .intersection(unit)

        let candidates = [
            CGRect(x: 0, y: 0, width: excluded.minX, height: 1),
            CGRect(x: excluded.maxX, y: 0, width: 1 - excluded.maxX, height: 1),
            CGRect(x: excluded.minX, y: 0,
                   width: excluded.width, height: excluded.minY),
            CGRect(x: excluded.minX, y: excluded.maxY,
                   width: excluded.width, height: 1 - excluded.maxY),
        ]
        let tiles = candidates.filter {
            $0.width > 0 && $0.height > 0
                && $0.width * $0.height >= minimumTileAreaFraction
        }
        let background = tiles.reduce(0.0) {
            $0 + Double($1.width * $1.height)
        }
        return CameraBackgroundRegionPlan(
            excludedPersonRegion: excluded,
            tiles: tiles,
            backgroundFraction: background
        )
    }
}

struct CameraTextureQualityEvidence: Equatable, Sendable {
    /// Smallest eigenvalue of the mean 2D structure tensor, using luma scaled to
    /// 0...1. A one-dimensional edge/stripe has an eigenvalue near zero even if
    /// its ordinary luma standard deviation is large.
    let minimumStructureTensorEigenvalue: Double
}

struct CameraValidatedVisionTranslation: Equatable, Sendable {
    let translationPixels: SIMD2<Double>
    let roundedPixelShift: SIMD2<Int>
}

/// Converts Vision's affine result into the strictly translational evidence
/// this adapter is calibrated to consume. Bounding the rounded shift by the
/// raster dimensions both rejects a zero-overlap result and makes every later
/// integer offset/addition safe.
enum CameraVisionTranslationPolicy {
    static func validate(
        _ transform: CGAffineTransform,
        imageSizePixels: SIMD2<Int>
    ) -> CameraValidatedVisionTranslation? {
        guard imageSizePixels.x > 0, imageSizePixels.y > 0,
              imageSizePixels.x <= Int.max / 2,
              imageSizePixels.y <= Int.max / 2,
              transform.a.isFinite, transform.b.isFinite,
              transform.c.isFinite, transform.d.isFinite,
              transform.tx.isFinite, transform.ty.isFinite,
              transform.a == 1, transform.b == 0,
              transform.c == 0, transform.d == 1 else { return nil }
        let translation = SIMD2(Double(transform.tx), Double(transform.ty))
        guard abs(translation.x) <= Double(imageSizePixels.x),
              abs(translation.y) <= Double(imageSizePixels.y),
              let shiftX = Int(exactly: translation.x.rounded()),
              let shiftY = Int(exactly: translation.y.rounded()) else {
            return nil
        }
        return CameraValidatedVisionTranslation(
            translationPixels: translation,
            roundedPixelShift: SIMD2(shiftX, shiftY)
        )
    }
}

enum CameraPlannedTileEvidencePolicy {
    static func isComplete(
        plannedTileCount: Int,
        validatedRegistrationCount: Int
    ) -> Bool {
        plannedTileCount > 0
            && validatedRegistrationCount == plannedTileCount
    }
}

struct CameraFixedBoxAverageGridPlan: Equatable, Sendable {
    let xOriginsPixels: [Int]
    let yOriginsPixels: [Int]
    let boxSizePixels: Int
    let boxSpacingPixels: Int
    let roundedTargetShiftPixels: SIMD2<Int>
    let usablePixelArea: Int

    /// Vision's alignment transform maps target coordinates into reference
    /// coordinates: reference = target + shift. Sampling the matching target
    /// box therefore subtracts the validated shift from the reference origin.
    func targetBoxOriginPixels(
        forReferenceBoxOriginPixels reference: SIMD2<Int>
    ) -> SIMD2<Int>? {
        let (x, xOverflow) = reference.x.subtractingReportingOverflow(
            roundedTargetShiftPixels.x
        )
        let (y, yOverflow) = reference.y.subtractingReportingOverflow(
            roundedTargetShiftPixels.y
        )
        guard !xOverflow, !yOverflow else { return nil }
        return SIMD2(x, y)
    }
}

/// Defines one render-pixel-anchored sampling lattice for both images. The
/// target boxes are translated by Vision's validated integer shift; neither the
/// box size nor its spacing changes with tile area or source resolution.
enum CameraFixedBoxAverageGridPolicy {
    static func make(
        imageSizePixels: SIMD2<Int>,
        normalizedTile: CGRect,
        roundedPixelShift: SIMD2<Int>,
        boxSizePixels: Int,
        boxSpacingPixels: Int
    ) -> CameraFixedBoxAverageGridPlan? {
        guard imageSizePixels.x >= 2, imageSizePixels.y >= 2,
              imageSizePixels.x <= Int.max / 2,
              imageSizePixels.y <= Int.max / 2,
              roundedPixelShift.x >= -imageSizePixels.x,
              roundedPixelShift.x <= imageSizePixels.x,
              roundedPixelShift.y >= -imageSizePixels.y,
              roundedPixelShift.y <= imageSizePixels.y,
              normalizedTile.origin.x.isFinite,
              normalizedTile.origin.y.isFinite,
              normalizedTile.width.isFinite,
              normalizedTile.height.isFinite,
              boxSizePixels > 0,
              boxSpacingPixels >= boxSizePixels else { return nil }
        let unit = CGRect(x: 0, y: 0, width: 1, height: 1)
        let tile = normalizedTile.standardized.intersection(unit)
        guard !tile.isNull, !tile.isEmpty,
              tile.minX.isFinite, tile.maxX.isFinite,
              tile.minY.isFinite, tile.maxY.isFinite else { return nil }

        let width = imageSizePixels.x
        let height = imageSizePixels.y
        let tileX0 = max(0, Int((tile.minX * Double(width)).rounded(.up)))
        let tileX1 = min(width, Int((tile.maxX * Double(width)).rounded(.down)))
        let tileY0 = max(0, Int((tile.minY * Double(height)).rounded(.up)))
        let tileY1 = min(height, Int((tile.maxY * Double(height)).rounded(.down)))
        let shiftX = roundedPixelShift.x
        let shiftY = roundedPixelShift.y
        let x0 = max(tileX0, shiftX)
        let x1 = min(tileX1, width + shiftX)
        let y0 = max(tileY0, shiftY)
        let y1 = min(tileY1, height + shiftY)
        guard x1 > x0, y1 > y0,
              x1 - x0 <= Int.max / (y1 - y0),
              let xOrigins = alignedOrigins(
                  lowerBound: x0,
                  upperBound: x1,
                  boxSizePixels: boxSizePixels,
                  spacingPixels: boxSpacingPixels
              ),
              let yOrigins = alignedOrigins(
                  lowerBound: y0,
                  upperBound: y1,
                  boxSizePixels: boxSizePixels,
                  spacingPixels: boxSpacingPixels
              ),
              xOrigins.count >= 5,
              yOrigins.count >= 5,
              xOrigins.count <= Int.max / boxSizePixels,
              yOrigins.count <= Int.max / boxSizePixels else { return nil }
        let sampledWidth = xOrigins.count * boxSizePixels
        let sampledHeight = yOrigins.count * boxSizePixels
        guard sampledWidth <= Int.max / sampledHeight else { return nil }
        return CameraFixedBoxAverageGridPlan(
            xOriginsPixels: xOrigins,
            yOriginsPixels: yOrigins,
            boxSizePixels: boxSizePixels,
            boxSpacingPixels: boxSpacingPixels,
            roundedTargetShiftPixels: roundedPixelShift,
            // Spacing is at least box size, so these boxes never overlap. Only
            // their measured union is credited as usable background evidence.
            usablePixelArea: sampledWidth * sampledHeight
        )
    }

    private static func alignedOrigins(
        lowerBound: Int,
        upperBound: Int,
        boxSizePixels: Int,
        spacingPixels: Int
    ) -> [Int]? {
        guard lowerBound >= 0,
              upperBound >= lowerBound,
              upperBound - lowerBound >= boxSizePixels else { return nil }
        let remainder = lowerBound % spacingPixels
        let delta = remainder == 0 ? 0 : spacingPixels - remainder
        guard lowerBound <= Int.max - delta else { return nil }
        let first = lowerBound + delta
        let last = upperBound - boxSizePixels
        guard first <= last else { return nil }
        return Array(stride(from: first, through: last, by: spacingPixels))
    }
}

struct CameraFixedBoxAveragePair: Equatable, Sendable {
    let referenceBoxAverages: [Double]
    let targetBoxAverages: [Double]
    let width: Int
    let height: Int
    let renderOriginPixels: SIMD2<Int>
    let boxSpacingPixels: Int
    let usablePixelArea: Int
}

/// Executes the fixed box averages from a validated plan. Pixel access remains
/// injected so the same arithmetic serves synthetic luma tests and locked BGRA
/// buffers without copying a full render raster.
enum CameraFixedBoxAveragePairSampler {
    static func sample(
        plan: CameraFixedBoxAverageGridPlan,
        referenceLumaAt: (_ x: Int, _ y: Int) -> Double?,
        targetLumaAt: (_ x: Int, _ y: Int) -> Double?
    ) -> CameraFixedBoxAveragePair? {
        guard !plan.xOriginsPixels.isEmpty,
              !plan.yOriginsPixels.isEmpty,
              plan.boxSizePixels > 0,
              plan.xOriginsPixels.count
                <= Int.max / plan.yOriginsPixels.count,
              plan.boxSizePixels <= Int.max / plan.boxSizePixels else {
            return nil
        }
        let count = plan.xOriginsPixels.count * plan.yOriginsPixels.count
        let pixelsPerBox = plan.boxSizePixels * plan.boxSizePixels
        var referenceAverages: [Double] = []
        var targetAverages: [Double] = []
        referenceAverages.reserveCapacity(count)
        targetAverages.reserveCapacity(count)
        for yOrigin in plan.yOriginsPixels {
            for xOrigin in plan.xOriginsPixels {
                guard let targetOrigin = plan.targetBoxOriginPixels(
                    forReferenceBoxOriginPixels: SIMD2(xOrigin, yOrigin)
                ) else { return nil }
                var referenceSum = 0.0
                var targetSum = 0.0
                for boxY in 0..<plan.boxSizePixels {
                    for boxX in 0..<plan.boxSizePixels {
                        guard let first = referenceLumaAt(
                                  xOrigin + boxX, yOrigin + boxY
                              ), let second = targetLumaAt(
                                  targetOrigin.x + boxX,
                                  targetOrigin.y + boxY
                              ), first.isFinite, second.isFinite else {
                            return nil
                        }
                        referenceSum += first
                        targetSum += second
                    }
                }
                let firstAverage = referenceSum / Double(pixelsPerBox)
                let secondAverage = targetSum / Double(pixelsPerBox)
                guard firstAverage.isFinite, secondAverage.isFinite else {
                    return nil
                }
                referenceAverages.append(firstAverage)
                targetAverages.append(secondAverage)
            }
        }
        guard referenceAverages.count == count,
              targetAverages.count == count,
              let originX = plan.xOriginsPixels.first,
              let originY = plan.yOriginsPixels.first else { return nil }
        return CameraFixedBoxAveragePair(
            referenceBoxAverages: referenceAverages,
            targetBoxAverages: targetAverages,
            width: plan.xOriginsPixels.count,
            height: plan.yOriginsPixels.count,
            renderOriginPixels: SIMD2(originX, originY),
            boxSpacingPixels: plan.boxSpacingPixels,
            usablePixelArea: plan.usablePixelArea
        )
    }
}

struct CameraRegistrationPeakEvidence: Equatable, Sendable {
    let bestOffsetPixels: SIMD2<Int>
    let bestCorrelation: Double
    /// Strongest distinct peak on the fine shared-domain surface. Global
    /// matched-domain scores are used only for the normalized gap because
    /// their domains differ and their raw values are not directly comparable.
    let secondBestCorrelation: Double
    /// Minimum of the fine-surface and exhaustive matched-domain global gaps,
    /// clamped to the calibrated 0...1 quality domain.
    let normalizedPeakSeparation: Double
}

struct CameraRegistrationCorrelationPlan: Equatable, Sendable {
    let fineCandidateCount: Int
    let finePairCountPerCandidate: Int
    let fineCorrelationPairCount: Int
    let zeroLagPrefixPairCount: Int
    let broadAliasCandidateCount: Int
    let tailAliasCandidateCount: Int
    let totalCorrelationPairCount: Int
}

/// Measures ambiguity from the actual reference/target registration surface.
/// Candidate shifts are a fixed render-pixel lattice around Vision's claimed
/// alignment. The claimed zero offset must be the global peak, and a distinct
/// periodic peak lowers the separation instead of being hidden by one-image
/// texture statistics.
enum CameraRegistrationPeakAnalyzer {
    /// Hard operation bound for one tile's complete 2D correlation surface.
    /// This constant is part of the calibration fingerprint.
    static let maximumCorrelationPairCountPerTile = 500_000
    static let aliasMinimumOverlapPairCount = 64
    static let aliasSharedDomainSideSamples = 8
    static let aliasTailPairCount = 48

    /// Keeps malformed or uncalibrated grids from turning even cost planning
    /// into an unbounded loop. Valid production grids are at most 512 samples
    /// on either axis (4,096 render pixels at the 8 px box lattice).
    private static let maximumPlannableGridDimensionSamples = 16_384

    static func hasCompleteCandidateSupport(
        width: Int,
        height: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        maximumSearchRadiusPixels: Int
    ) -> Bool {
        makeCorrelationPlan(
            width: width,
            height: height,
            renderOriginPixels: renderOriginPixels,
            boxSpacingPixels: boxSpacingPixels,
            correlationSampleSpacingPixels: correlationSampleSpacingPixels,
            maximumSearchRadiusPixels: maximumSearchRadiusPixels
        ) != nil
    }

    /// A globally phased child can only remove fine-domain samples. If an
    /// unsupported parent cannot furnish the calibrated 64 fine pairs, no
    /// recursive descendant can become a valid analysis leaf.
    static func hasMinimumFineCandidateSupport(
        width: Int,
        height: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        maximumSearchRadiusPixels: Int
    ) -> Bool {
        guard width >= 5, height >= 5,
              renderOriginPixels.x >= 0, renderOriginPixels.y >= 0,
              boxSpacingPixels > 0,
              renderOriginPixels.x.isMultiple(of: boxSpacingPixels),
              renderOriginPixels.y.isMultiple(of: boxSpacingPixels),
              correlationSampleSpacingPixels >= boxSpacingPixels,
              correlationSampleSpacingPixels.isMultiple(of: boxSpacingPixels),
              maximumSearchRadiusPixels >= boxSpacingPixels,
              maximumSearchRadiusPixels.isMultiple(of: boxSpacingPixels) else {
            return false
        }
        let radius = maximumSearchRadiusPixels / boxSpacingPixels
        let sampleStride = correlationSampleSpacingPixels / boxSpacingPixels
        guard radius <= 64, sampleStride <= 64,
              let pairCount = pairedSampleCount(
                  width: width,
                  height: height,
                  commonInsetSamples: radius,
                  renderOriginPixels: renderOriginPixels,
                  boxSpacingPixels: boxSpacingPixels,
                  sampleSpacingPixels: correlationSampleSpacingPixels,
                  sampleStride: sampleStride
              ) else { return false }
        return pairCount >= aliasMinimumOverlapPairCount
    }

    static func makeCorrelationPlan(
        width: Int,
        height: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        maximumSearchRadiusPixels: Int
    ) -> CameraRegistrationCorrelationPlan? {
        guard width >= 5, height >= 5,
              width <= maximumPlannableGridDimensionSamples,
              height <= maximumPlannableGridDimensionSamples,
              renderOriginPixels.x >= 0, renderOriginPixels.y >= 0,
              boxSpacingPixels > 0,
              renderOriginPixels.x.isMultiple(of: boxSpacingPixels),
              renderOriginPixels.y.isMultiple(of: boxSpacingPixels),
              correlationSampleSpacingPixels >= boxSpacingPixels,
              correlationSampleSpacingPixels.isMultiple(of: boxSpacingPixels),
              maximumSearchRadiusPixels >= boxSpacingPixels,
              maximumSearchRadiusPixels.isMultiple(of: boxSpacingPixels) else {
            return nil
        }
        let radius = maximumSearchRadiusPixels / boxSpacingPixels
        let sampleStride = correlationSampleSpacingPixels / boxSpacingPixels
        guard radius <= 64, sampleStride <= 64 else { return nil }
        let side = radius * 2 + 1
        let candidateCount = side * side
        guard let pairCount = pairedSampleCount(
            width: width,
            height: height,
            commonInsetSamples: radius,
            renderOriginPixels: renderOriginPixels,
            boxSpacingPixels: boxSpacingPixels,
            sampleSpacingPixels: correlationSampleSpacingPixels,
            sampleStride: sampleStride
        ), pairCount >= aliasMinimumOverlapPairCount,
           let fineCost = checkedMultiply(pairCount, candidateCount),
           let prefixCost = checkedMultiply(width, height),
           let validOffsetCount = supportedAliasOffsetCount(
               width: width,
               height: height
           ) else { return nil }

        let sharedSide = aliasSharedDomainSideSamples
        guard width >= sharedSide, height >= sharedSide else { return nil }
        let maximumBroadX = width - sharedSide
        let maximumBroadY = height - sharedSide
        // The count subtraction below is a set difference, not merely integer
        // arithmetic: every fine offset must lie inside the broad rectangle.
        // A narrow/high grid can have a larger broad *count* while missing the
        // fine rectangle's x columns, which would double-enumerate candidates
        // and make the runtime receipt disagree with this plan.
        guard maximumBroadX >= radius, maximumBroadY >= radius else {
            return nil
        }
        guard let broadWidth = checkedAdd(
                  checkedMultiply(maximumBroadX, 2), 1
              ), let broadHeight = checkedAdd(
                  checkedMultiply(maximumBroadY, 2), 1
              ), let broadOffsetCount = checkedMultiply(
                  broadWidth, broadHeight
              ), broadOffsetCount >= candidateCount else { return nil }
        let broadAliasCandidateCount = broadOffsetCount - candidateCount
        guard validOffsetCount >= broadOffsetCount else { return nil }
        let tailAliasCandidateCount = validOffsetCount - broadOffsetCount
        guard let broadCost = checkedMultiply(
                  broadAliasCandidateCount,
                  sharedSide * sharedSide
              ), let tailCost = checkedMultiply(
                  tailAliasCandidateCount,
                  aliasTailPairCount
              ), let fineAndPrefix = checkedAdd(fineCost, prefixCost),
              let withBroad = checkedAdd(fineAndPrefix, broadCost),
              let totalCost = checkedAdd(withBroad, tailCost),
              totalCost <= maximumCorrelationPairCountPerTile else {
            return nil
        }
        return CameraRegistrationCorrelationPlan(
            fineCandidateCount: candidateCount,
            finePairCountPerCandidate: pairCount,
            fineCorrelationPairCount: fineCost,
            zeroLagPrefixPairCount: prefixCost,
            broadAliasCandidateCount: broadAliasCandidateCount,
            tailAliasCandidateCount: tailAliasCandidateCount,
            totalCorrelationPairCount: totalCost
        )
    }

    static func analyze(
        referenceBoxAverages: [Double],
        targetBoxAverages: [Double],
        width: Int,
        height: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        maximumSearchRadiusPixels: Int,
        minimumPeakSeparationPixels: Int,
        minimumBestCorrelation: Double
    ) -> CameraRegistrationPeakEvidence? {
        guard width >= 5, height >= 5,
              width <= referenceBoxAverages.count / height,
              width * height == referenceBoxAverages.count,
              targetBoxAverages.count == referenceBoxAverages.count,
              referenceBoxAverages.allSatisfy(\.isFinite),
              targetBoxAverages.allSatisfy(\.isFinite),
              renderOriginPixels.x >= 0, renderOriginPixels.y >= 0,
              boxSpacingPixels > 0,
              renderOriginPixels.x.isMultiple(of: boxSpacingPixels),
              renderOriginPixels.y.isMultiple(of: boxSpacingPixels),
              correlationSampleSpacingPixels >= boxSpacingPixels,
              correlationSampleSpacingPixels.isMultiple(of: boxSpacingPixels),
              maximumSearchRadiusPixels >= boxSpacingPixels,
              maximumSearchRadiusPixels.isMultiple(of: boxSpacingPixels),
              minimumPeakSeparationPixels >= boxSpacingPixels,
              minimumPeakSeparationPixels <= maximumSearchRadiusPixels,
              minimumPeakSeparationPixels.isMultiple(of: boxSpacingPixels),
              minimumBestCorrelation.isFinite,
              (0...1).contains(minimumBestCorrelation),
              let correlationPlan = makeCorrelationPlan(
                  width: width,
                  height: height,
                  renderOriginPixels: renderOriginPixels,
                  boxSpacingPixels: boxSpacingPixels,
                  correlationSampleSpacingPixels:
                      correlationSampleSpacingPixels,
                  maximumSearchRadiusPixels: maximumSearchRadiusPixels
              ) else {
            return nil
        }

        let radius = maximumSearchRadiusPixels / boxSpacingPixels
        let minimumSeparation = minimumPeakSeparationPixels / boxSpacingPixels
        let sampleStride = correlationSampleSpacingPixels / boxSpacingPixels
        // A malformed fingerprint must not turn the peak surface into an
        // unbounded quadratic search.
        guard radius <= 64, sampleStride <= 64 else { return nil }
        let side = radius * 2 + 1
        var correlations = Array<Double?>(repeating: nil, count: side * side)
        var evaluatedPairCount = 0
        for offsetY in -radius...radius {
            for offsetX in -radius...radius {
                guard let computation = normalizedCorrelation(
                    reference: referenceBoxAverages,
                    target: targetBoxAverages,
                    width: width,
                    height: height,
                    offsetX: offsetX,
                    offsetY: offsetY,
                    renderOriginPixels: renderOriginPixels,
                    boxSpacingPixels: boxSpacingPixels,
                    sampleSpacingPixels: correlationSampleSpacingPixels,
                    sampleStride: sampleStride,
                    commonInsetSamples: radius
                ), computation.pairCount
                    == correlationPlan.finePairCountPerCandidate,
                   let nextPairCount = checkedAdd(
                       evaluatedPairCount,
                       computation.pairCount
                   )
                else { return nil }
                evaluatedPairCount = nextPairCount
                correlations[(offsetY + radius) * side + offsetX + radius]
                    = computation.score
            }
        }
        guard evaluatedPairCount
                == correlationPlan.fineCorrelationPairCount else { return nil }

        func correlation(atX x: Int, y: Int) -> Double? {
            guard (-radius...radius).contains(x),
                  (-radius...radius).contains(y) else { return nil }
            return correlations[(y + radius) * side + x + radius]
        }
        func isLocalPeak(x: Int, y: Int, score: Double) -> Bool {
            for neighborY in max(-radius, y - 1)...min(radius, y + 1) {
                for neighborX in max(-radius, x - 1)...min(radius, x + 1) {
                    if neighborX == x && neighborY == y { continue }
                    if let neighbor = correlation(atX: neighborX, y: neighborY),
                       neighbor > score + 1e-12 {
                        return false
                    }
                }
            }
            return true
        }

        guard let best = correlation(atX: 0, y: 0),
              best >= minimumBestCorrelation,
              isLocalPeak(x: 0, y: 0, score: best),
              correlations.compactMap({ $0 }).allSatisfy({
                  $0 <= best + 1e-12
              }) else { return nil }
        var secondBest = -1.0
        for offsetY in -radius...radius {
            for offsetX in -radius...radius {
                guard max(abs(offsetX), abs(offsetY)) >= minimumSeparation,
                      let score = correlation(atX: offsetX, y: offsetY),
                      isLocalPeak(x: offsetX, y: offsetY, score: score) else {
                    continue
                }
                secondBest = max(secondBest, score)
            }
        }
        let fineSeparation = max(0, min(1, best - secondBest))
        guard best.isFinite, secondBest.isFinite,
              fineSeparation.isFinite,
              let zeroLagIntegral = ZeroLagCorrelationIntegral(
                  reference: referenceBoxAverages,
                  target: targetBoxAverages,
                  width: width,
                  height: height
              ), zeroLagIntegral.pairCount
                    == correlationPlan.zeroLagPrefixPairCount,
              let withPrefix = checkedAdd(
                  evaluatedPairCount,
                  zeroLagIntegral.pairCount
              ) else { return nil }
        evaluatedPairCount = withPrefix

        var globalSeparation = 1.0
        func consumeMatchedDomain(
            zeroScore: Double,
            candidateScore: Double
        ) -> Bool {
            guard zeroScore.isFinite, candidateScore.isFinite,
                  candidateScore <= zeroScore + 1e-12 else { return false }
            let gap = max(0, min(1, zeroScore - candidateScore))
            guard gap.isFinite else { return false }
            globalSeparation = min(globalSeparation, gap)
            return true
        }

        let sharedSide = aliasSharedDomainSideSamples
        let maximumBroadX = width - sharedSide
        let maximumBroadY = height - sharedSide
        var broadCandidateCount = 0
        var broadZeroScores = Array<Double?>(repeating: nil, count: 4)
        for offsetY in -maximumBroadY...maximumBroadY {
            for offsetX in -maximumBroadX...maximumBroadX {
                guard max(abs(offsetX), abs(offsetY)) > radius else { continue }
                let quadrant = aliasQuadrant(offsetX: offsetX, offsetY: offsetY)
                let domain = broadAliasDomain(
                    quadrant: quadrant,
                    width: width,
                    height: height
                )
                let zeroScore: Double
                if let cached = broadZeroScores[quadrant] {
                    zeroScore = cached
                } else {
                    guard let score = zeroLagIntegral.correlation(in: domain) else {
                        return nil
                    }
                    broadZeroScores[quadrant] = score
                    zeroScore = score
                }
                guard let candidate = shiftedCorrelation(
                    reference: referenceBoxAverages,
                    target: targetBoxAverages,
                    width: width,
                    domain: domain,
                    offsetX: offsetX,
                    offsetY: offsetY,
                    integral: zeroLagIntegral
                ), candidate.pairCount == sharedSide * sharedSide,
                   consumeMatchedDomain(
                       zeroScore: zeroScore,
                       candidateScore: candidate.score
                   ), let nextPairCount = checkedAdd(
                       evaluatedPairCount,
                       candidate.pairCount
                   ) else { return nil }
                evaluatedPairCount = nextPairCount
                broadCandidateCount += 1
            }
        }

        var tailCandidateCount = 0
        for offsetY in -(height - 1)...(height - 1) {
            for offsetX in -(width - 1)...(width - 1) {
                guard abs(offsetX) > maximumBroadX
                        || abs(offsetY) > maximumBroadY,
                      let overlap = overlapDomain(
                          width: width,
                          height: height,
                          offsetX: offsetX,
                          offsetY: offsetY
                      ), overlap.area >= aliasMinimumOverlapPairCount else {
                    continue
                }
                guard let domain = tailAliasDomain(in: overlap),
                      domain.area == aliasTailPairCount,
                      let zeroScore = zeroLagIntegral.correlation(in: domain),
                      let candidate = shiftedCorrelation(
                          reference: referenceBoxAverages,
                          target: targetBoxAverages,
                          width: width,
                          domain: domain,
                          offsetX: offsetX,
                          offsetY: offsetY,
                          integral: zeroLagIntegral
                      ), candidate.pairCount == aliasTailPairCount,
                      consumeMatchedDomain(
                          zeroScore: zeroScore,
                          candidateScore: candidate.score
                      ), let nextPairCount = checkedAdd(
                          evaluatedPairCount,
                          candidate.pairCount
                      ) else { return nil }
                evaluatedPairCount = nextPairCount
                tailCandidateCount += 1
            }
        }
        guard broadCandidateCount
                == correlationPlan.broadAliasCandidateCount,
              tailCandidateCount == correlationPlan.tailAliasCandidateCount,
              evaluatedPairCount
                == correlationPlan.totalCorrelationPairCount else { return nil }
        let separation = min(fineSeparation, globalSeparation)
        return CameraRegistrationPeakEvidence(
            bestOffsetPixels: .zero,
            bestCorrelation: best,
            secondBestCorrelation: secondBest,
            normalizedPeakSeparation: separation
        )
    }

    private struct CorrelationComputation {
        let score: Double
        let pairCount: Int
    }

    private static func normalizedCorrelation(
        reference: [Double],
        target: [Double],
        width: Int,
        height: Int,
        offsetX: Int,
        offsetY: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        sampleSpacingPixels: Int,
        sampleStride: Int,
        commonInsetSamples: Int
    ) -> CorrelationComputation? {
        guard pairedSampleCount(
            width: width,
            height: height,
            commonInsetSamples: commonInsetSamples,
            renderOriginPixels: renderOriginPixels,
            boxSpacingPixels: boxSpacingPixels,
            sampleSpacingPixels: sampleSpacingPixels,
            sampleStride: sampleStride
        ) != nil else { return nil }
        let startX = commonInsetSamples
        let endX = width - commonInsetSamples
        let startY = commonInsetSamples
        let endY = height - commonInsetSamples
        guard let firstX = firstSampleIndex(
                  atOrAfter: startX,
                  renderOriginPixels: renderOriginPixels.x,
                  boxSpacingPixels: boxSpacingPixels,
                  sampleSpacingPixels: sampleSpacingPixels
              ),
              let firstY = firstSampleIndex(
                  atOrAfter: startY,
                  renderOriginPixels: renderOriginPixels.y,
                  boxSpacingPixels: boxSpacingPixels,
                  sampleSpacingPixels: sampleSpacingPixels
              ),
              firstX < endX, firstY < endY else { return nil }

        var count = 0.0
        var firstSum = 0.0
        var secondSum = 0.0
        var firstSquared = 0.0
        var secondSquared = 0.0
        var productSum = 0.0
        var row = firstY
        while row < endY {
            var column = firstX
            while column < endX {
                let first = reference[row * width + column]
                let second = target[
                    (row + offsetY) * width + column + offsetX
                ]
                count += 1
                firstSum += first
                secondSum += second
                firstSquared += first * first
                secondSquared += second * second
                productSum += first * second
                column += sampleStride
            }
            row += sampleStride
        }
        guard count >= 64 else { return nil }
        let covariance = productSum - firstSum * secondSum / count
        let firstVariance = firstSquared - firstSum * firstSum / count
        let secondVariance = secondSquared - secondSum * secondSum / count
        let denominator = sqrt(max(0, firstVariance * secondVariance))
        guard denominator > 1e-12 else { return nil }
        return CorrelationComputation(
            score: max(-1, min(1, covariance / denominator)),
            pairCount: Int(count)
        )
    }

    private struct AliasDomain {
        let x: Int
        let y: Int
        let width: Int
        let height: Int

        var area: Int { width * height }

        func translated(x offsetX: Int, y offsetY: Int) -> AliasDomain {
            AliasDomain(
                x: x + offsetX,
                y: y + offsetY,
                width: width,
                height: height
            )
        }
    }

    private struct ZeroLagCorrelationIntegral {
        let width: Int
        let height: Int
        let prefixStride: Int
        let referenceSum: [Double]
        let targetSum: [Double]
        let referenceSquaredSum: [Double]
        let targetSquaredSum: [Double]
        let zeroProductSum: [Double]
        let pairCount: Int

        init?(
            reference: [Double],
            target: [Double],
            width: Int,
            height: Int
        ) {
            guard width > 0, height > 0,
                  width <= reference.count / height,
                  width * height == reference.count,
                  target.count == reference.count,
                  width < Int.max, height < Int.max else { return nil }
            let stride = width + 1
            guard height + 1 <= Int.max / stride else { return nil }
            let prefixCount = stride * (height + 1)
            var referenceSum = Array(repeating: 0.0, count: prefixCount)
            var targetSum = Array(repeating: 0.0, count: prefixCount)
            var referenceSquared = Array(repeating: 0.0, count: prefixCount)
            var targetSquared = Array(repeating: 0.0, count: prefixCount)
            var product = Array(repeating: 0.0, count: prefixCount)
            for y in 0..<height {
                var referenceRowSum = 0.0
                var targetRowSum = 0.0
                var referenceSquaredRowSum = 0.0
                var targetSquaredRowSum = 0.0
                var productRowSum = 0.0
                for x in 0..<width {
                    let first = reference[y * width + x]
                    let second = target[y * width + x]
                    referenceRowSum += first
                    targetRowSum += second
                    referenceSquaredRowSum += first * first
                    targetSquaredRowSum += second * second
                    productRowSum += first * second
                    let index = (y + 1) * stride + x + 1
                    let above = index - stride
                    referenceSum[index] = referenceSum[above] + referenceRowSum
                    targetSum[index] = targetSum[above] + targetRowSum
                    referenceSquared[index] = referenceSquared[above]
                        + referenceSquaredRowSum
                    targetSquared[index] = targetSquared[above]
                        + targetSquaredRowSum
                    product[index] = product[above] + productRowSum
                }
            }
            self.width = width
            self.height = height
            self.prefixStride = stride
            self.referenceSum = referenceSum
            self.targetSum = targetSum
            self.referenceSquaredSum = referenceSquared
            self.targetSquaredSum = targetSquared
            self.zeroProductSum = product
            self.pairCount = width * height
        }

        func correlation(in domain: AliasDomain) -> Double? {
            guard contains(domain) else { return nil }
            return CameraRegistrationPeakAnalyzer.correlationScore(
                count: domain.area,
                firstSum: sum(referenceSum, in: domain),
                secondSum: sum(targetSum, in: domain),
                firstSquared: sum(referenceSquaredSum, in: domain),
                secondSquared: sum(targetSquaredSum, in: domain),
                productSum: sum(zeroProductSum, in: domain)
            )
        }

        func contains(_ domain: AliasDomain) -> Bool {
            domain.x >= 0 && domain.y >= 0
                && domain.width > 0 && domain.height > 0
                && domain.x <= width - domain.width
                && domain.y <= height - domain.height
        }

        func referenceMoments(
            in domain: AliasDomain
        ) -> (sum: Double, squared: Double)? {
            guard contains(domain) else { return nil }
            return (
                sum(referenceSum, in: domain),
                sum(referenceSquaredSum, in: domain)
            )
        }

        func targetMoments(
            in domain: AliasDomain
        ) -> (sum: Double, squared: Double)? {
            guard contains(domain) else { return nil }
            return (
                sum(targetSum, in: domain),
                sum(targetSquaredSum, in: domain)
            )
        }

        private func sum(
            _ prefix: [Double],
            in domain: AliasDomain
        ) -> Double {
            let x0 = domain.x
            let y0 = domain.y
            let x1 = x0 + domain.width
            let y1 = y0 + domain.height
            return prefix[y1 * prefixStride + x1]
                - prefix[y0 * prefixStride + x1]
                - prefix[y1 * prefixStride + x0]
                + prefix[y0 * prefixStride + x0]
        }
    }

    private static func shiftedCorrelation(
        reference: [Double],
        target: [Double],
        width: Int,
        domain: AliasDomain,
        offsetX: Int,
        offsetY: Int,
        integral: ZeroLagCorrelationIntegral
    ) -> CorrelationComputation? {
        let shiftedDomain = domain.translated(x: offsetX, y: offsetY)
        guard integral.contains(domain), integral.contains(shiftedDomain),
              let first = integral.referenceMoments(in: domain),
              let second = integral.targetMoments(in: shiftedDomain) else {
            return nil
        }
        var product = 0.0
        var pairCount = 0
        for y in domain.y..<(domain.y + domain.height) {
            for x in domain.x..<(domain.x + domain.width) {
                product += reference[y * width + x]
                    * target[(y + offsetY) * width + x + offsetX]
                pairCount += 1
            }
        }
        guard pairCount == domain.area,
              let score = correlationScore(
                  count: pairCount,
                  firstSum: first.sum,
                  secondSum: second.sum,
                  firstSquared: first.squared,
                  secondSquared: second.squared,
                  productSum: product
              ) else { return nil }
        return CorrelationComputation(score: score, pairCount: pairCount)
    }

    private static func correlationScore(
        count: Int,
        firstSum: Double,
        secondSum: Double,
        firstSquared: Double,
        secondSquared: Double,
        productSum: Double
    ) -> Double? {
        guard count > 1, firstSum.isFinite, secondSum.isFinite,
              firstSquared.isFinite, secondSquared.isFinite,
              productSum.isFinite else { return nil }
        let sampleCount = Double(count)
        let covariance = productSum - firstSum * secondSum / sampleCount
        let firstVariance = firstSquared - firstSum * firstSum / sampleCount
        let secondVariance = secondSquared - secondSum * secondSum / sampleCount
        let denominator = sqrt(max(0, firstVariance * secondVariance))
        guard denominator > 1e-12 else { return nil }
        return max(-1, min(1, covariance / denominator))
    }

    private static func aliasQuadrant(
        offsetX: Int,
        offsetY: Int
    ) -> Int {
        (offsetX < 0 ? 1 : 0) | (offsetY < 0 ? 2 : 0)
    }

    private static func broadAliasDomain(
        quadrant: Int,
        width: Int,
        height: Int
    ) -> AliasDomain {
        let side = aliasSharedDomainSideSamples
        return AliasDomain(
            x: quadrant & 1 == 0 ? 0 : width - side,
            y: quadrant & 2 == 0 ? 0 : height - side,
            width: side,
            height: side
        )
    }

    private static func overlapDomain(
        width: Int,
        height: Int,
        offsetX: Int,
        offsetY: Int
    ) -> AliasDomain? {
        let x0 = max(0, -offsetX)
        let y0 = max(0, -offsetY)
        let x1 = min(width, width - offsetX)
        let y1 = min(height, height - offsetY)
        guard x1 > x0, y1 > y0 else { return nil }
        return AliasDomain(x: x0, y: y0, width: x1 - x0, height: y1 - y0)
    }

    static func tailAliasDomainDimensions(
        overlapWidth: Int,
        overlapHeight: Int
    ) -> SIMD2<Int>? {
        guard overlapWidth > 0, overlapHeight > 0,
              overlapWidth <= Int.max / overlapHeight,
              overlapWidth * overlapHeight
                >= aliasMinimumOverlapPairCount else { return nil }
        let factors = [
            SIMD2(1, 48), SIMD2(2, 24), SIMD2(3, 16), SIMD2(4, 12),
            SIMD2(6, 8), SIMD2(8, 6), SIMD2(12, 4), SIMD2(16, 3),
            SIMD2(24, 2), SIMD2(48, 1),
        ]
        let overlapAspect = Double(overlapWidth) / Double(overlapHeight)
        return factors.filter {
            $0.x <= overlapWidth && $0.y <= overlapHeight
        }.min {
            let firstDistance = abs(Double($0.x) / Double($0.y) - overlapAspect)
            let secondDistance = abs(Double($1.x) / Double($1.y) - overlapAspect)
            if firstDistance == secondDistance { return $0.x < $1.x }
            return firstDistance < secondDistance
        }
    }

    private static func tailAliasDomain(
        in overlap: AliasDomain
    ) -> AliasDomain? {
        guard let dimensions = tailAliasDomainDimensions(
            overlapWidth: overlap.width,
            overlapHeight: overlap.height
        ) else { return nil }
        return AliasDomain(
            x: overlap.x + (overlap.width - dimensions.x) / 2,
            y: overlap.y + (overlap.height - dimensions.y) / 2,
            width: dimensions.x,
            height: dimensions.y
        )
    }

    private static func supportedAliasOffsetCount(
        width: Int,
        height: Int
    ) -> Int? {
        var total = 0
        for absoluteX in 0..<width {
            let overlapWidth = width - absoluteX
            let requiredHeight = (
                aliasMinimumOverlapPairCount + overlapWidth - 1
            ) / overlapWidth
            guard requiredHeight <= height else { continue }
            let maximumAbsoluteY = height - requiredHeight
            guard let signedYCount = checkedAdd(
                      checkedMultiply(maximumAbsoluteY, 2), 1
                  ), let contribution = checkedMultiply(
                      signedYCount,
                      absoluteX == 0 ? 1 : 2
                  ), let next = checkedAdd(total, contribution) else {
                return nil
            }
            total = next
        }
        return total
    }

    private static func checkedMultiply(_ lhs: Int, _ rhs: Int) -> Int? {
        let result = lhs.multipliedReportingOverflow(by: rhs)
        return result.overflow ? nil : result.partialValue
    }

    private static func checkedAdd(_ lhs: Int, _ rhs: Int) -> Int? {
        let result = lhs.addingReportingOverflow(rhs)
        return result.overflow ? nil : result.partialValue
    }

    private static func checkedAdd(_ lhs: Int?, _ rhs: Int) -> Int? {
        guard let lhs else { return nil }
        return checkedAdd(lhs, rhs)
    }

    private static func pairedSampleCount(
        width: Int,
        height: Int,
        commonInsetSamples: Int,
        renderOriginPixels: SIMD2<Int>,
        boxSpacingPixels: Int,
        sampleSpacingPixels: Int,
        sampleStride: Int
    ) -> Int? {
        guard width > 0, height > 0, sampleStride > 0,
              commonInsetSamples >= 0,
              commonInsetSamples < width,
              commonInsetSamples < height else { return nil }
        let startX = commonInsetSamples
        let endX = width - commonInsetSamples
        let startY = commonInsetSamples
        let endY = height - commonInsetSamples
        guard let firstX = firstSampleIndex(
                  atOrAfter: startX,
                  renderOriginPixels: renderOriginPixels.x,
                  boxSpacingPixels: boxSpacingPixels,
                  sampleSpacingPixels: sampleSpacingPixels
              ),
              let firstY = firstSampleIndex(
                  atOrAfter: startY,
                  renderOriginPixels: renderOriginPixels.y,
                  boxSpacingPixels: boxSpacingPixels,
                  sampleSpacingPixels: sampleSpacingPixels
              ),
              firstX < endX, firstY < endY else { return nil }
        let xCount = (endX - 1 - firstX) / sampleStride + 1
        let yCount = (endY - 1 - firstY) / sampleStride + 1
        guard xCount > 0, yCount > 0,
              xCount <= Int.max / yCount else { return nil }
        return xCount * yCount
    }

    private static func firstSampleIndex(
        atOrAfter lowerBound: Int,
        renderOriginPixels: Int,
        boxSpacingPixels: Int,
        sampleSpacingPixels: Int
    ) -> Int? {
        guard lowerBound >= 0,
              renderOriginPixels >= 0,
              boxSpacingPixels > 0,
              lowerBound <= Int.max / boxSpacingPixels else { return nil }
        let offsetPixels = lowerBound * boxSpacingPixels
        guard renderOriginPixels <= Int.max - offsetPixels else { return nil }
        let position = renderOriginPixels + offsetPixels
        let remainder = position % sampleSpacingPixels
        let deltaPixels = remainder == 0 ? 0 : sampleSpacingPixels - remainder
        guard deltaPixels.isMultiple(of: boxSpacingPixels) else { return nil }
        let deltaSamples = deltaPixels / boxSpacingPixels
        guard lowerBound <= Int.max - deltaSamples else { return nil }
        return lowerBound + deltaSamples
    }
}

/// Readiness must bind the A knobs to an actually runnable calibrated raster,
/// not merely validate each integer in isolation.
enum CameraCalibrationAppearanceGeometryPolicy {
    static func isSupported(
        frameSizePixels: SIMD2<Int>,
        boxAverageSizePixels: Int,
        boxAverageSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        peakSearchRadiusPixels: Int,
        peakMinimumSeparationPixels: Int,
        minimumTileAreaFraction: Double
    ) -> Bool {
        guard boxAverageSpacingPixels > 0,
              minimumTileAreaFraction.isFinite,
              (0...1).contains(minimumTileAreaFraction),
              peakMinimumSeparationPixels >= boxAverageSpacingPixels,
              peakMinimumSeparationPixels <= peakSearchRadiusPixels,
              peakMinimumSeparationPixels.isMultiple(
                  of: boxAverageSpacingPixels
              ),
              let plan = CameraBackgroundAnalysisTilePlanner.make(
                  backgroundTiles: [CGRect(x: 0, y: 0, width: 1, height: 1)],
                  imageSizePixels: frameSizePixels,
                  minimumTileAreaFraction: minimumTileAreaFraction,
                  boxAverageSizePixels: boxAverageSizePixels,
                  boxAverageSpacingPixels: boxAverageSpacingPixels,
                  correlationSampleSpacingPixels:
                      correlationSampleSpacingPixels,
                  peakSearchRadiusPixels: peakSearchRadiusPixels
              ), !plan.tiles.isEmpty,
              plan.tiles.count
                <= CameraBackgroundAnalysisTilePlanner.maximumAnalysisTileCount
        else { return false }
        return true
    }
}

struct CameraBackgroundAnalysisTilePlan: Equatable, Sendable {
    let tiles: [CGRect]
    let sampledBackgroundFraction: Double
}

/// Removes tiles that cannot support the complete fingerprinted peak surface
/// before any Vision request. Over-budget regions are carved into deterministic
/// globally aligned leaves; unsupported remainders are not credited as sampled
/// background. A bounded recursive split handles smaller irregular regions.
enum CameraBackgroundAnalysisTilePlanner {
    static let maximumAnalysisTileCount = 16

    static func make(
        backgroundTiles: [CGRect],
        imageSizePixels: SIMD2<Int>,
        minimumTileAreaFraction: Double,
        boxAverageSizePixels: Int,
        boxAverageSpacingPixels: Int,
        correlationSampleSpacingPixels: Int,
        peakSearchRadiusPixels: Int
    ) -> CameraBackgroundAnalysisTilePlan? {
        guard !backgroundTiles.isEmpty,
              backgroundTiles.count <= maximumAnalysisTileCount,
              imageSizePixels.x >= 2, imageSizePixels.y >= 2,
              minimumTileAreaFraction.isFinite,
              (0...1).contains(minimumTileAreaFraction),
              imageSizePixels.x <= Int.max / imageSizePixels.y else {
            return nil
        }
        let frameArea = imageSizePixels.x * imageSizePixels.y
        let unit = CGRect(x: 0, y: 0, width: 1, height: 1)
        guard backgroundTiles.allSatisfy({
            $0.origin.x.isFinite && $0.origin.y.isFinite
                && $0.width.isFinite && $0.height.isFinite
                && $0.width > 0 && $0.height > 0 && unit.contains($0)
        }) else { return nil }
        for left in backgroundTiles.indices {
            for right in backgroundTiles.indices where left < right {
                let overlap = backgroundTiles[left]
                    .intersection(backgroundTiles[right])
                guard overlap.isNull || overlap.isEmpty else { return nil }
            }
        }

        func evidence(
            for tile: CGRect
        ) -> (tile: CGRect, sampledArea: Int)? {
            let normalizedArea = Double(tile.width * tile.height)
            guard normalizedArea.isFinite,
                  normalizedArea >= minimumTileAreaFraction,
                  let grid = CameraFixedBoxAverageGridPolicy.make(
                      imageSizePixels: imageSizePixels,
                      normalizedTile: tile,
                      roundedPixelShift: .zero,
                      boxSizePixels: boxAverageSizePixels,
                      boxSpacingPixels: boxAverageSpacingPixels
                  ), let originX = grid.xOriginsPixels.first,
                  let originY = grid.yOriginsPixels.first,
                  CameraRegistrationPeakAnalyzer.hasCompleteCandidateSupport(
                      width: grid.xOriginsPixels.count,
                      height: grid.yOriginsPixels.count,
                      renderOriginPixels: SIMD2(originX, originY),
                      boxSpacingPixels: boxAverageSpacingPixels,
                      correlationSampleSpacingPixels:
                          correlationSampleSpacingPixels,
                      maximumSearchRadiusPixels: peakSearchRadiusPixels
                  ) else { return nil }
            return (tile, grid.usablePixelArea)
        }

        /// 48x45 box grids retain at least 72 fine-surface pairs at the
        /// production 32 px correlation phase while leaving enough of the
        /// 500k budget for the exhaustive alias screen. Using the grid's actual
        /// origins keeps every carved leaf globally phase-aligned.
        func carvedLeaves(
            from tile: CGRect
        ) -> [(tile: CGRect, sampledArea: Int)] {
            let leafWidthSamples = 48
            let leafHeightSamples = 45
            guard let grid = CameraFixedBoxAverageGridPolicy.make(
                      imageSizePixels: imageSizePixels,
                      normalizedTile: tile,
                      roundedPixelShift: .zero,
                      boxSizePixels: boxAverageSizePixels,
                      boxSpacingPixels: boxAverageSpacingPixels
                  ) else { return [] }
            let columnCount = grid.xOriginsPixels.count / leafWidthSamples
            let rowCount = grid.yOriginsPixels.count / leafHeightSamples
            guard columnCount > 0, rowCount > 0,
                  columnCount <= maximumAnalysisTileCount / rowCount else {
                return []
            }
            var leaves: [(tile: CGRect, sampledArea: Int)] = []
            leaves.reserveCapacity(columnCount * rowCount)
            for row in 0..<rowCount {
                let firstY = grid.yOriginsPixels[row * leafHeightSamples]
                let lastY = grid.yOriginsPixels[
                    (row + 1) * leafHeightSamples - 1
                ]
                for column in 0..<columnCount {
                    let firstX = grid.xOriginsPixels[
                        column * leafWidthSamples
                    ]
                    let lastX = grid.xOriginsPixels[
                        (column + 1) * leafWidthSamples - 1
                    ]
                    let pixelWidth = lastX + boxAverageSizePixels - firstX
                    let pixelHeight = lastY + boxAverageSizePixels - firstY
                    let child = CGRect(
                        x: Double(firstX) / Double(imageSizePixels.x),
                        y: Double(firstY) / Double(imageSizePixels.y),
                        width: Double(pixelWidth) / Double(imageSizePixels.x),
                        height: Double(pixelHeight) / Double(imageSizePixels.y)
                    )
                    guard tile.contains(child), let leaf = evidence(for: child) else {
                        return []
                    }
                    leaves.append(leaf)
                }
            }
            return leaves
        }

        func halves(of tile: CGRect) -> [CGRect] {
            let pixelWidth = Double(tile.width) * Double(imageSizePixels.x)
            let pixelHeight = Double(tile.height) * Double(imageSizePixels.y)
            if pixelWidth >= pixelHeight {
                let half = tile.width / 2
                return [
                    CGRect(x: tile.minX, y: tile.minY,
                           width: half, height: tile.height),
                    CGRect(x: tile.minX + half, y: tile.minY,
                           width: tile.width - half, height: tile.height),
                ]
            }
            let half = tile.height / 2
            return [
                CGRect(x: tile.minX, y: tile.minY,
                       width: tile.width, height: half),
                CGRect(x: tile.minX, y: tile.minY + half,
                       width: tile.width, height: tile.height - half),
            ]
        }

        var exceededTileCount = false
        func partition(
            _ tile: CGRect,
            depth: Int
        ) -> [(tile: CGRect, sampledArea: Int)] {
            guard !exceededTileCount, depth <= maximumAnalysisTileCount else {
                exceededTileCount = true
                return []
            }
            let parent = evidence(for: tile)
            if parent == nil {
                let carved = carvedLeaves(from: tile)
                if !carved.isEmpty { return carved }
                guard let grid = CameraFixedBoxAverageGridPolicy.make(
                          imageSizePixels: imageSizePixels,
                          normalizedTile: tile,
                          roundedPixelShift: .zero,
                          boxSizePixels: boxAverageSizePixels,
                          boxSpacingPixels: boxAverageSpacingPixels
                      ), let originX = grid.xOriginsPixels.first,
                      let originY = grid.yOriginsPixels.first,
                      CameraRegistrationPeakAnalyzer
                          .hasMinimumFineCandidateSupport(
                              width: grid.xOriginsPixels.count,
                              height: grid.yOriginsPixels.count,
                              renderOriginPixels: SIMD2(originX, originY),
                              boxSpacingPixels: boxAverageSpacingPixels,
                              correlationSampleSpacingPixels:
                                  correlationSampleSpacingPixels,
                              maximumSearchRadiusPixels:
                                  peakSearchRadiusPixels
                          ) else { return [] }
            }
            let children = halves(of: tile)
            let directChildren = children.compactMap { evidence(for: $0) }
            if directChildren.count == 2 {
                return directChildren
            }
            if let parent { return [parent] }

            var leaves: [(tile: CGRect, sampledArea: Int)] = []
            for child in children {
                leaves.append(contentsOf: partition(child, depth: depth + 1))
                if leaves.count > maximumAnalysisTileCount {
                    exceededTileCount = true
                    return []
                }
            }
            return leaves
        }

        var selected: [(tile: CGRect, sampledArea: Int)] = []
        selected.reserveCapacity(maximumAnalysisTileCount)
        for tile in backgroundTiles {
            selected.append(contentsOf: partition(tile, depth: 0))
            guard !exceededTileCount,
                  selected.count <= maximumAnalysisTileCount else { return nil }
        }
        var sampledArea = 0
        for item in selected {
            // Selected tiles/children are disjoint. This simultaneously keeps
            // credited coverage <= one frame and box averaging <= two luma
            // reads per credited pixel (one per image) for the whole estimate.
            guard item.sampledArea <= frameArea - sampledArea else { return nil }
            sampledArea += item.sampledArea
        }
        return CameraBackgroundAnalysisTilePlan(
            tiles: selected.map(\.tile),
            sampledBackgroundFraction:
                Double(sampledArea) / Double(frameArea)
        )
    }
}

/// Pure two-dimensional texture evidence. Registration ambiguity deliberately
/// lives in `CameraRegistrationPeakAnalyzer`, where both images are observed.
enum CameraTextureQualityAnalyzer {
    static func analyze(
        luma: [Double],
        width: Int,
        height: Int
    ) -> CameraTextureQualityEvidence? {
        guard width >= 5, height >= 5,
              width <= luma.count / height,
              width * height == luma.count,
              luma.allSatisfy(\.isFinite) else { return nil }

        var xx = 0.0
        var xy = 0.0
        var yy = 0.0
        var gradientCount = 0.0
        for row in 1..<(height - 1) {
            for column in 1..<(width - 1) {
                let index = row * width + column
                let gx = (luma[index + 1] - luma[index - 1]) / 510
                let gy = (luma[index + width] - luma[index - width]) / 510
                xx += gx * gx
                xy += gx * gy
                yy += gy * gy
                gradientCount += 1
            }
        }
        guard gradientCount > 0 else { return nil }
        xx /= gradientCount
        xy /= gradientCount
        yy /= gradientCount
        let trace = xx + yy
        let discriminant = max(0, (xx - yy) * (xx - yy) + 4 * xy * xy)
        let minimumEigenvalue = max(0, (trace - sqrt(discriminant)) / 2)
        guard minimumEigenvalue.isFinite else { return nil }
        return CameraTextureQualityEvidence(
            minimumStructureTensorEigenvalue: minimumEigenvalue
        )
    }
}

struct CameraTileRegistration: Equatable, Sendable {
    /// Tile centre relative to the image centre, in upright-image pixels.
    let centrePixels: SIMD2<Double>
    /// Translation that maps the target tile back onto the reference tile.
    let translationPixels: SIMD2<Double>
    /// Non-overlapping area of this tile as a fraction of the upright image.
    let areaFraction: Double
    /// Adapter-derived identifiability and post-warp evidence. Vision confidence
    /// is deliberately absent because it may be a meaningless constant 1.
    let minimumStructureTensorEigenvalue: Double
    let registrationUniqueness: Double
    let postWarpMeanAbsoluteError: Double
}

struct CameraTileMotionFit: Equatable, Sendable {
    let translationPixels: SIMD2<Double>
    let normalizedTranslation: SIMD2<Double>
    let rotationRadians: Double
    let scaleFraction: Double
    let fitResidualPixels: Double
    let registrationQuality: Double
    let usableBackgroundFraction: Double
    let usedTileCount: Int
}

/// Fits the small-angle similarity image field
/// `u = tx - theta*y + scale*x`, `v = ty + theta*x + scale*y`
/// across disjoint background tiles.
/// This makes in-plane rotation observable instead of collapsing every tile to
/// one global translation. A high residual is treated as weak evidence (for
/// example parallax or foreground leakage), never as zero camera motion.
enum CameraTileMotionFitter {
    struct Configuration: Equatable, Sendable {
        let minimumTileCount: Int
        let minimumStructureTensorEigenvalue: Double
        let minimumRegistrationUniqueness: Double
        let maximumPostWarpMeanAbsoluteError: Double
        let maximumFitResidualPixels: Double
        /// Minimum smaller eigenvalue of the weighted tile-centre covariance,
        /// divided by image diagonal squared. This rejects collinear/clustered
        /// tiles that cannot robustly distinguish 2D rotation and scale.
        let minimumSpatialEigenvalueFraction: Double
    }

    static func fit(
        _ registrations: [CameraTileRegistration],
        imageDiagonalPixels: Double,
        configuration: Configuration
    ) -> CameraTileMotionFit? {
        guard valid(configuration), imageDiagonalPixels.isFinite,
              imageDiagonalPixels > 0 else { return nil }
        let usable = registrations.filter {
            individuallyUsable($0, configuration: configuration)
        }
        guard usable.count >= configuration.minimumTileCount else { return nil }
        guard let solution = solve(
            usable,
            imageDiagonalPixels: imageDiagonalPixels,
            configuration: configuration
        ), solution.maximumResidual
            <= configuration.maximumFitResidualPixels else { return nil }
        let usableArea = usable.reduce(0.0) { $0 + $1.areaFraction }
        guard usableArea.isFinite, usableArea > 0,
              usableArea <= 1 + 1e-9 else { return nil }
        let fitQuality = max(
            0,
            1 - solution.fitResidual
                / (2 * configuration.maximumFitResidualPixels)
        )
        let quality = min(
            solution.minimumTileQuality,
            solution.geometryQuality,
            fitQuality
        )
        guard quality.isFinite else { return nil }
        return CameraTileMotionFit(
            translationPixels: solution.translation,
            normalizedTranslation: solution.translation / imageDiagonalPixels,
            rotationRadians: solution.rotation,
            scaleFraction: solution.scale,
            fitResidualPixels: solution.fitResidual,
            registrationQuality: quality,
            usableBackgroundFraction: min(1, usableArea),
            usedTileCount: usable.count
        )
    }

    static func individuallyUsable(
        _ registration: CameraTileRegistration,
        configuration: Configuration
    ) -> Bool {
        registration.centrePixels.x.isFinite
            && registration.centrePixels.y.isFinite
            && registration.translationPixels.x.isFinite
            && registration.translationPixels.y.isFinite
            && registration.areaFraction.isFinite
            && registration.areaFraction > 0
            && registration.areaFraction <= 1
            && registration.minimumStructureTensorEigenvalue.isFinite
            && registration.minimumStructureTensorEigenvalue
                >= configuration.minimumStructureTensorEigenvalue
            && registration.registrationUniqueness.isFinite
            && registration.registrationUniqueness
                >= configuration.minimumRegistrationUniqueness
            && registration.registrationUniqueness <= 1
            && registration.postWarpMeanAbsoluteError.isFinite
            && registration.postWarpMeanAbsoluteError >= 0
            && registration.postWarpMeanAbsoluteError
                <= configuration.maximumPostWarpMeanAbsoluteError
    }

    private struct Solution {
        let translation: SIMD2<Double>
        let rotation: Double
        let scale: Double
        let residuals: [Double]
        let fitResidual: Double
        let maximumResidual: Double
        let minimumTileQuality: Double
        let geometryQuality: Double
    }

    private static func solve(
        _ registrations: [CameraTileRegistration],
        imageDiagonalPixels: Double,
        configuration: Configuration
    ) -> Solution? {
        let qualities = registrations.map {
            tileQuality($0, configuration: configuration)
        }
        let weights = zip(registrations, qualities).map {
            $0.areaFraction * $1
        }
        let weightSum = weights.reduce(0, +)
        guard weightSum.isFinite, weightSum > 0 else { return nil }
        let centreMean = zip(registrations, weights).reduce(SIMD2<Double>.zero) {
            $0 + $1.0.centrePixels * $1.1
        } / weightSum
        let translationMean = zip(registrations, weights).reduce(SIMD2<Double>.zero) {
            $0 + $1.0.translationPixels * $1.1
        } / weightSum

        var covarianceXX = 0.0
        var covarianceXY = 0.0
        var covarianceYY = 0.0
        var rotationNumerator = 0.0
        var scaleNumerator = 0.0
        var denominator = 0.0
        for (registration, weight) in zip(registrations, weights) {
            let centre = registration.centrePixels - centreMean
            let displacement = registration.translationPixels - translationMean
            covarianceXX += weight * centre.x * centre.x
            covarianceXY += weight * centre.x * centre.y
            covarianceYY += weight * centre.y * centre.y
            rotationNumerator += weight
                * (-centre.y * displacement.x + centre.x * displacement.y)
            scaleNumerator += weight
                * (centre.x * displacement.x + centre.y * displacement.y)
            denominator += weight * simd_length_squared(centre)
        }
        covarianceXX /= weightSum
        covarianceXY /= weightSum
        covarianceYY /= weightSum
        let covarianceTrace = covarianceXX + covarianceYY
        let covarianceDiscriminant = max(
            0,
            (covarianceXX - covarianceYY) * (covarianceXX - covarianceYY)
                + 4 * covarianceXY * covarianceXY
        )
        let spatialEigenvalue = max(
            0,
            (covarianceTrace - sqrt(covarianceDiscriminant)) / 2
        )
        let minimumSpatialEigenvalue = configuration.minimumSpatialEigenvalueFraction
            * imageDiagonalPixels * imageDiagonalPixels
        guard spatialEigenvalue >= minimumSpatialEigenvalue,
              denominator.isFinite, denominator > 1e-12 else { return nil }

        let rotation = rotationNumerator / denominator
        let scale = scaleNumerator / denominator
        let translation = SIMD2(
            translationMean.x + rotation * centreMean.y - scale * centreMean.x,
            translationMean.y - rotation * centreMean.x - scale * centreMean.y
        )
        let residuals = registrations.map { registration in
            let predicted = SIMD2(
                translation.x - rotation * registration.centrePixels.y
                    + scale * registration.centrePixels.x,
                translation.y + rotation * registration.centrePixels.x
                    + scale * registration.centrePixels.y
            )
            return simd_length(registration.translationPixels - predicted)
        }
        let weightedSquaredResidual = zip(residuals, weights).reduce(0.0) {
            $0 + $1.0 * $1.0 * $1.1
        }
        let fitResidual = sqrt(weightedSquaredResidual / weightSum)
        let geometryQuality = min(
            spatialEigenvalue / (2 * minimumSpatialEigenvalue),
            1
        )
        guard rotation.isFinite, scale.isFinite,
              translation.x.isFinite, translation.y.isFinite,
              fitResidual.isFinite,
              let maximumResidual = residuals.max(),
              let minimumTileQuality = qualities.min() else { return nil }
        return Solution(
            translation: translation,
            rotation: rotation,
            scale: scale,
            residuals: residuals,
            fitResidual: fitResidual,
            maximumResidual: maximumResidual,
            minimumTileQuality: minimumTileQuality,
            geometryQuality: geometryQuality
        )
    }

    private static func tileQuality(
        _ registration: CameraTileRegistration,
        configuration: Configuration
    ) -> Double {
        let textureQuality = min(
            registration.minimumStructureTensorEigenvalue
                / (2 * configuration.minimumStructureTensorEigenvalue),
            1
        )
        let uniquenessQuality = min(
            registration.registrationUniqueness
                / (2 * configuration.minimumRegistrationUniqueness),
            1
        )
        let appearanceQuality = max(
            0,
            1 - registration.postWarpMeanAbsoluteError
                / (2 * configuration.maximumPostWarpMeanAbsoluteError)
        )
        return min(textureQuality, uniquenessQuality, appearanceQuality)
    }

    private static func valid(_ configuration: Configuration) -> Bool {
        configuration.minimumTileCount >= 3
            && configuration.minimumStructureTensorEigenvalue.isFinite
            && configuration.minimumStructureTensorEigenvalue > 0
            && configuration.minimumRegistrationUniqueness.isFinite
            && configuration.minimumRegistrationUniqueness > 0
            && configuration.minimumRegistrationUniqueness <= 1
            && configuration.maximumPostWarpMeanAbsoluteError.isFinite
            && configuration.maximumPostWarpMeanAbsoluteError > 0
            && configuration.maximumFitResidualPixels.isFinite
            && configuration.maximumFitResidualPixels > 0
            && configuration.minimumSpatialEigenvalueFraction.isFinite
            && configuration.minimumSpatialEigenvalueFraction > 0
            && configuration.minimumSpatialEigenvalueFraction < 1
    }
}

struct CameraAnchoredMotionEstimate: Equatable, Sendable {
    let baselineTimestamp: TimeInterval
    let fit: CameraTileMotionFit
}

struct CameraAnchorMotionPeaks: Equatable, Sendable {
    let translation: CameraAnchoredMotionEstimate
    let rotation: CameraAnchoredMotionEstimate
    let scale: CameraAnchoredMotionEstimate
}

enum CameraAnchorPeakSelector {
    static func select(
        _ estimates: [CameraAnchoredMotionEstimate]
    ) -> CameraAnchorMotionPeaks? {
        guard estimates.allSatisfy({ $0.baselineTimestamp.isFinite }),
              let translation = estimates.max(by: {
                  simd_length($0.fit.normalizedTranslation)
                      < simd_length($1.fit.normalizedTranslation)
              }),
              let rotation = estimates.max(by: {
                  abs($0.fit.rotationRadians) < abs($1.fit.rotationRadians)
              }),
              let scale = estimates.max(by: {
                  abs($0.fit.scaleFraction) < abs($1.fit.scaleFraction)
              }) else { return nil }
        return CameraAnchorMotionPeaks(
            translation: translation,
            rotation: rotation,
            scale: scale
        )
    }
}

/// Pure, deterministic product policy over native-frame registration results.
/// Vision and AVFoundation live below this boundary; neither may silently turn
/// a missing or low-quality observation into zero camera motion.
enum CameraMotionReducer {
    struct Configuration: Equatable, Sendable {
        let derivativeWindowSeconds: TimeInterval
        /// Per-clip allowance derived from actual native PTS, not the profile's
        /// hard cap and never nominal frame-rate metadata.
        let allowedNativeFrameGapSeconds: TimeInterval
        let robustNativeFrameIntervalSeconds: TimeInterval
        let staticMaximumNormalizedTranslation: Double
        let movingMinimumNormalizedTranslation: Double
        let staticMaximumRotationRadians: Double
        let movingMinimumRotationRadians: Double
        let staticMaximumScaleFraction: Double
        let movingMinimumScaleFraction: Double
        let minimumRegistrationQuality: Double
        let minimumRegistrationPeakCorrelation: Double
        let minimumBackgroundFraction: Double
        /// Upright post-composition dimensions used for these observations.
        /// They are part of the calibration identity, not incidental metadata.
        let analysisFrameSizePixels: SIMD2<Int>
        let maximumAnalysisPixelCount: Int
        let maximumAnalysisDimensionPixels: Int
        let maximumAnalysisAspectRatio: Double
        let maximumRetainedPixelBufferBytes: Int
        let appearanceBoxAverageSizePixels: Int
        let appearanceBoxAverageSpacingPixels: Int
        let registrationCorrelationSampleSpacingPixels: Int
        let registrationPeakSearchRadiusPixels: Int
        let registrationPeakMinimumSeparationPixels: Int
        let calibrationFingerprint: CameraMotionCalibrationFingerprint?
    }

    static func reduce(
        observations: [CameraMotionObservation],
        configuration: Configuration
    ) -> CameraReferenceState {
        guard valid(configuration), !observations.isEmpty else {
            return observations.isEmpty
                ? .indeterminate(.insufficientMeasurements)
                : .indeterminate(.invalidMeasurement)
        }

        let timestampTolerance = 1e-9
        var peakTranslation = 0.0
        var peakRotation = 0.0
        var peakScale = 0.0

        for (index, observation) in observations.enumerated() {
            guard observation.startTimestamp.isFinite,
                  observation.endTimestamp.isFinite,
                  observation.translationWindowBaselineTimestamp.isFinite,
                  observation.rotationWindowBaselineTimestamp.isFinite,
                  observation.scaleWindowBaselineTimestamp.isFinite,
                  observation.endTimestamp > observation.startTimestamp,
                  observation.translationWindowBaselineTimestamp
                    <= observation.startTimestamp + timestampTolerance,
                  observation.rotationWindowBaselineTimestamp
                    <= observation.startTimestamp + timestampTolerance,
                  observation.scaleWindowBaselineTimestamp
                    <= observation.startTimestamp + timestampTolerance,
                  observation.normalizedStepTranslation.x.isFinite,
                  observation.normalizedStepTranslation.y.isFinite,
                  observation.normalizedWindowTranslation.x.isFinite,
                  observation.normalizedWindowTranslation.y.isFinite,
                  observation.stepRotationRadians.isFinite,
                  observation.windowRotationRadians.isFinite,
                  observation.stepScaleFraction.isFinite,
                  observation.windowScaleFraction.isFinite,
                  observation.registrationQuality.isFinite,
                  (0...1).contains(observation.registrationQuality),
                  observation.backgroundFraction.isFinite,
                  (0...1).contains(observation.backgroundFraction) else {
                return .indeterminate(.invalidMeasurement)
            }

            let interval = observation.endTimestamp - observation.startTimestamp
            let baselineAges = [
                observation.endTimestamp
                    - observation.translationWindowBaselineTimestamp,
                observation.endTimestamp
                    - observation.rotationWindowBaselineTimestamp,
                observation.endTimestamp
                    - observation.scaleWindowBaselineTimestamp,
            ]
            guard interval
                    <= configuration.allowedNativeFrameGapSeconds
                        + timestampTolerance,
                  baselineAges.allSatisfy({
                      $0 >= -timestampTolerance
                          && $0 <= configuration.derivativeWindowSeconds
                            + timestampTolerance
                  }) else {
                return .indeterminate(.timestampDiscontinuity)
            }
            if index > 0,
               abs(observation.startTimestamp - observations[index - 1].endTimestamp)
                > timestampTolerance {
                return .indeterminate(.timestampDiscontinuity)
            }
            guard observation.registrationQuality
                    >= configuration.minimumRegistrationQuality else {
                return .indeterminate(.lowRegistrationQuality)
            }
            guard observation.backgroundFraction
                    >= configuration.minimumBackgroundFraction else {
                return .indeterminate(.insufficientBackground)
            }

            peakTranslation = max(
                peakTranslation,
                simd_length(observation.normalizedStepTranslation),
                simd_length(observation.normalizedWindowTranslation)
            )
            peakRotation = max(
                peakRotation,
                abs(observation.stepRotationRadians),
                abs(observation.windowRotationRadians)
            )
            peakScale = max(
                peakScale,
                abs(observation.stepScaleFraction),
                abs(observation.windowScaleFraction)
            )
        }

        guard observations.count >= 2 else {
            return .indeterminate(.insufficientMeasurements)
        }
        let analyzedDuration = observations.last!.endTimestamp
            - observations.first!.startTimestamp
        guard analyzedDuration + timestampTolerance
                >= configuration.derivativeWindowSeconds else {
            return .indeterminate(.insufficientCoverage)
        }

        guard peakTranslation.isFinite, peakRotation.isFinite,
              peakScale.isFinite else {
            return .indeterminate(.invalidMeasurement)
        }
        let evidence = CameraMotionEvidence(
            analyzedDurationSeconds: analyzedDuration,
            derivativeWindowSeconds: configuration.derivativeWindowSeconds,
            peakNormalizedTranslation: peakTranslation,
            translationStaticUpperBound: configuration.staticMaximumNormalizedTranslation,
            translationMovingLowerBound: configuration.movingMinimumNormalizedTranslation,
            peakRotationRadians: peakRotation,
            rotationStaticUpperBoundRadians: configuration.staticMaximumRotationRadians,
            rotationMovingLowerBoundRadians: configuration.movingMinimumRotationRadians,
            peakScaleFraction: peakScale,
            scaleStaticUpperBound: configuration.staticMaximumScaleFraction,
            scaleMovingLowerBound: configuration.movingMinimumScaleFraction,
            calibrationProfileID: configuration.calibrationFingerprint?.profileID,
            analysisFrameWidthPixels: configuration.analysisFrameSizePixels.x,
            analysisFrameHeightPixels: configuration.analysisFrameSizePixels.y,
            maximumAnalysisPixelCount: configuration.maximumAnalysisPixelCount,
            measuredIntervals: observations.count,
            sampledFrames: observations.count + 1
        )
        guard calibrationProfileMatches(
            configuration,
            observations: observations
        ) else {
            return .calibrationRequired(evidence)
        }
        if peakTranslation + 1e-12
                >= configuration.movingMinimumNormalizedTranslation
            || peakRotation + 1e-12 >= configuration.movingMinimumRotationRadians
            || peakScale + 1e-12 >= configuration.movingMinimumScaleFraction {
            return .moving(evidence)
        }
        if peakTranslation <= configuration.staticMaximumNormalizedTranslation
            && peakRotation <= configuration.staticMaximumRotationRadians
            && peakScale <= configuration.staticMaximumScaleFraction {
            return .staticWithinBudget(evidence)
        }
        return .betweenCalibrationBands(evidence)
    }

    /// The reducer is a second public safety boundary. It does not trust a bare
    /// profile label supplied by an orchestration caller: the typed fingerprint
    /// must bind every reducer parameter, the exact frame/window/cadence domain,
    /// this adapter revision, and the actual per-clip cadence allowance.
    private static func calibrationProfileMatches(
        _ configuration: Configuration,
        observations: [CameraMotionObservation]
    ) -> Bool {
        guard let profile = configuration.calibrationFingerprint,
              !profile.profileID
                .trimmingCharacters(in: .whitespacesAndNewlines).isEmpty,
              profile.adapterRevision
                == CameraMotionVideoAnalyzer.calibrationAdapterRevision,
              profile.humanRectanglesRequestRevision
                == CameraMotionVideoAnalyzer.humanRectanglesRequestRevision,
              profile.translationalRegistrationRequestRevision
                == CameraMotionVideoAnalyzer
                    .translationalRegistrationRequestRevision,
              CameraMotionVideoAnalyzer
                .pinnedVisionRequestRevisionsAreSupported,
              profile.analysisFrameSizePixels
                == configuration.analysisFrameSizePixels,
              profile.maximumAnalysisPixelCount
                == configuration.maximumAnalysisPixelCount,
              profile.maximumAnalysisDimensionPixels
                == configuration.maximumAnalysisDimensionPixels,
              profile.maximumAnalysisAspectRatio
                == configuration.maximumAnalysisAspectRatio,
              profile.maximumRetainedPixelBufferBytes
                == configuration.maximumRetainedPixelBufferBytes,
              profile.maximumRetainedPixelBufferCount
                == CameraAnalysisBufferBudget.maximumRetainedBufferCount,
              profile.appearanceBoxAverageSizePixels
                == configuration.appearanceBoxAverageSizePixels,
              profile.appearanceBoxAverageSpacingPixels
                == configuration.appearanceBoxAverageSpacingPixels,
              profile.registrationCorrelationSampleSpacingPixels
                == configuration.registrationCorrelationSampleSpacingPixels,
              profile.registrationPeakSearchRadiusPixels
                == configuration.registrationPeakSearchRadiusPixels,
              profile.registrationPeakMinimumSeparationPixels
                == configuration.registrationPeakMinimumSeparationPixels,
              profile.registrationAliasMinimumOverlapPairCount
                == CameraRegistrationPeakAnalyzer
                    .aliasMinimumOverlapPairCount,
              profile.registrationAliasSharedDomainSideSamples
                == CameraRegistrationPeakAnalyzer
                    .aliasSharedDomainSideSamples,
              profile.registrationAliasTailPairCount
                == CameraRegistrationPeakAnalyzer.aliasTailPairCount,
              profile.maximumRegistrationCorrelationPairCountPerTile
                == CameraRegistrationPeakAnalyzer
                    .maximumCorrelationPairCountPerTile,
              profile.maximumScanDurationSeconds
                == CameraMotionScanBudgetPolicy.maximumRangeDurationSeconds,
              profile.maximumNativeSampleCount
                == CameraMotionScanBudgetPolicy.maximumNativeSampleCount,
              profile.provisionalStaticTranslation
                == configuration.staticMaximumNormalizedTranslation,
              profile.provisionalMovingTranslation
                == configuration.movingMinimumNormalizedTranslation,
              profile.provisionalStaticRotationRadians
                == configuration.staticMaximumRotationRadians,
              profile.provisionalMovingRotationRadians
                == configuration.movingMinimumRotationRadians,
              profile.provisionalStaticScaleFraction
                == configuration.staticMaximumScaleFraction,
              profile.provisionalMovingScaleFraction
                == configuration.movingMinimumScaleFraction,
              profile.minimumRegistrationQuality
                == configuration.minimumRegistrationQuality,
              profile.minimumRegistrationPeakCorrelation
                == configuration.minimumRegistrationPeakCorrelation,
              profile.minimumBackgroundFraction
                == configuration.minimumBackgroundFraction,
              validCalibrationDomain(profile.derivativeWindowDomainSeconds),
              validCalibrationDomain(profile.nativeFrameIntervalDomainSeconds),
              calibrationDomain(
                profile.derivativeWindowDomainSeconds,
                contains: configuration.derivativeWindowSeconds
              ),
              profile.personBoxInflationFraction.isFinite,
              (0..<0.5).contains(profile.personBoxInflationFraction),
              profile.minimumTileAreaFraction.isFinite,
              (0...1).contains(profile.minimumTileAreaFraction),
              profile.minimumTileCount >= 3,
              profile.minimumStructureTensorEigenvalue.isFinite,
              profile.minimumStructureTensorEigenvalue >= 0,
              profile.minimumRegistrationUniqueness.isFinite,
              (0...1).contains(profile.minimumRegistrationUniqueness),
              profile.maximumPostWarpMeanAbsoluteError.isFinite,
              profile.maximumPostWarpMeanAbsoluteError > 0,
              profile.maximumFitResidualPixels.isFinite,
              profile.maximumFitResidualPixels > 0,
              profile.minimumSpatialEigenvalueFraction.isFinite,
              profile.minimumSpatialEigenvalueFraction > 0,
              let cadence = CameraMotionCadencePolicy.assess(
                nativeFrameIntervals: observations.map {
                    $0.endTimestamp - $0.startTimestamp
                },
                derivativeWindowSeconds: configuration.derivativeWindowSeconds,
                hardMaximumGapSeconds:
                    profile.maximumNativeFrameGapSeconds,
                gapMultiplier: profile.nativeFrameGapMultiplier,
                jitterAllowanceSeconds:
                    profile.nativeFrameJitterAllowanceSeconds
              ),
              calibrationDomain(
                profile.nativeFrameIntervalDomainSeconds,
                contains: cadence.robustNativeFrameIntervalSeconds
              ),
              abs(cadence.robustNativeFrameIntervalSeconds
                    - configuration.robustNativeFrameIntervalSeconds)
                <= calibrationTimeToleranceSeconds,
              abs(cadence.allowedNativeFrameGapSeconds
                    - configuration.allowedNativeFrameGapSeconds)
                <= calibrationTimeToleranceSeconds
        else { return false }
        return true
    }

    private static func validCalibrationDomain(
        _ domain: ClosedRange<TimeInterval>
    ) -> Bool {
        domain.lowerBound.isFinite
            && domain.upperBound.isFinite
            && domain.lowerBound > 0
            && domain.upperBound >= domain.lowerBound
    }

    /// Native PTS subtraction can differ from a profile's decimal cadence by
    /// one or two ULPs. Keep that representation noise distinct from a real
    /// calibration-domain mismatch.
    private static let calibrationTimeToleranceSeconds: TimeInterval = 1e-12

    private static func calibrationDomain(
        _ domain: ClosedRange<TimeInterval>,
        contains value: TimeInterval
    ) -> Bool {
        value.isFinite
            && value >= domain.lowerBound - calibrationTimeToleranceSeconds
            && value <= domain.upperBound + calibrationTimeToleranceSeconds
    }

    private static func valid(_ configuration: Configuration) -> Bool {
        configuration.derivativeWindowSeconds.isFinite
            && configuration.derivativeWindowSeconds > 0
            && configuration.allowedNativeFrameGapSeconds.isFinite
            && configuration.allowedNativeFrameGapSeconds > 0
            && configuration.allowedNativeFrameGapSeconds
                < configuration.derivativeWindowSeconds
            && configuration.robustNativeFrameIntervalSeconds.isFinite
            && configuration.robustNativeFrameIntervalSeconds > 0
            && configuration.staticMaximumNormalizedTranslation.isFinite
            && configuration.staticMaximumNormalizedTranslation >= 0
            && configuration.movingMinimumNormalizedTranslation.isFinite
            && configuration.movingMinimumNormalizedTranslation
                > configuration.staticMaximumNormalizedTranslation
            && configuration.staticMaximumRotationRadians.isFinite
            && configuration.staticMaximumRotationRadians >= 0
            && configuration.movingMinimumRotationRadians.isFinite
            && configuration.movingMinimumRotationRadians
                > configuration.staticMaximumRotationRadians
            && configuration.staticMaximumScaleFraction.isFinite
            && configuration.staticMaximumScaleFraction >= 0
            && configuration.movingMinimumScaleFraction.isFinite
            && configuration.movingMinimumScaleFraction
                > configuration.staticMaximumScaleFraction
            && configuration.minimumRegistrationQuality.isFinite
            && (0...1).contains(configuration.minimumRegistrationQuality)
            && configuration.minimumRegistrationPeakCorrelation.isFinite
            && (0...1).contains(
                configuration.minimumRegistrationPeakCorrelation
            )
            && configuration.minimumBackgroundFraction.isFinite
            && (0...1).contains(configuration.minimumBackgroundFraction)
            && configuration.analysisFrameSizePixels.x >= 2
            && configuration.analysisFrameSizePixels.y >= 2
            && configuration.analysisFrameSizePixels.x.isMultiple(of: 2)
            && configuration.analysisFrameSizePixels.y.isMultiple(of: 2)
            && configuration.analysisFrameSizePixels.x <= Int.max / 4
            && CameraAnalysisBufferBudget.isSupported(
                widthPixels: configuration.analysisFrameSizePixels.x,
                heightPixels: configuration.analysisFrameSizePixels.y,
                bytesPerRow: configuration.analysisFrameSizePixels.x * 4,
                maximumPixelCount: configuration.maximumAnalysisPixelCount,
                maximumDimensionPixels:
                    configuration.maximumAnalysisDimensionPixels,
                maximumAspectRatio: configuration.maximumAnalysisAspectRatio,
                maximumRetainedBytes:
                    configuration.maximumRetainedPixelBufferBytes
            )
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
            && CameraCalibrationAppearanceGeometryPolicy.isSupported(
                frameSizePixels: configuration.analysisFrameSizePixels,
                boxAverageSizePixels:
                    configuration.appearanceBoxAverageSizePixels,
                boxAverageSpacingPixels:
                    configuration.appearanceBoxAverageSpacingPixels,
                correlationSampleSpacingPixels:
                    configuration.registrationCorrelationSampleSpacingPixels,
                peakSearchRadiusPixels:
                    configuration.registrationPeakSearchRadiusPixels,
                peakMinimumSeparationPixels:
                    configuration.registrationPeakMinimumSeparationPixels,
                minimumTileAreaFraction:
                    configuration.calibrationFingerprint?
                        .minimumTileAreaFraction ?? 0.01
            )
            && (configuration.calibrationFingerprint.map {
                $0.analysisFrameSizePixels.x >= 2
                    && $0.analysisFrameSizePixels.y >= 2
                    && $0.analysisFrameSizePixels.x.isMultiple(of: 2)
                    && $0.analysisFrameSizePixels.y.isMultiple(of: 2)
                    && $0.maximumAnalysisDimensionPixels >= 2
                    && $0.maximumAnalysisAspectRatio.isFinite
                    && $0.maximumAnalysisAspectRatio >= 1
                    && $0.maximumRetainedPixelBufferBytes > 0
                    && $0.maximumRetainedPixelBufferCount > 0
                    && $0.appearanceBoxAverageSizePixels > 0
                    && $0.appearanceBoxAverageSpacingPixels
                        >= $0.appearanceBoxAverageSizePixels
                    && $0.registrationCorrelationSampleSpacingPixels
                        >= $0.appearanceBoxAverageSpacingPixels
                    && $0.registrationCorrelationSampleSpacingPixels
                        .isMultiple(of: $0.appearanceBoxAverageSpacingPixels)
                    && $0.registrationCorrelationSampleSpacingPixels
                        / $0.appearanceBoxAverageSpacingPixels <= 64
                    && $0.registrationPeakSearchRadiusPixels
                        >= $0.appearanceBoxAverageSpacingPixels
                    && $0.registrationPeakSearchRadiusPixels
                        .isMultiple(of: $0.appearanceBoxAverageSpacingPixels)
                    && $0.registrationPeakSearchRadiusPixels
                        / $0.appearanceBoxAverageSpacingPixels <= 64
                    && $0.registrationPeakMinimumSeparationPixels
                        >= $0.appearanceBoxAverageSpacingPixels
                    && $0.registrationPeakMinimumSeparationPixels
                        <= $0.registrationPeakSearchRadiusPixels
                    && $0.registrationPeakMinimumSeparationPixels
                        .isMultiple(of: $0.appearanceBoxAverageSpacingPixels)
                    && $0.registrationAliasMinimumOverlapPairCount > 0
                    && $0.registrationAliasSharedDomainSideSamples > 0
                    && $0.registrationAliasTailPairCount > 0
                    && $0.maximumRegistrationCorrelationPairCountPerTile > 0
            } ?? true)
    }
}
