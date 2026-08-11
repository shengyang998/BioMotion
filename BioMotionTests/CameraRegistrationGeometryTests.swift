import CoreGraphics
import simd
import XCTest

@testable import BioMotion

final class CameraRegistrationGeometryTests: XCTestCase {

    func testRenderGeometryRejectsScaleShearAndRankOnePreferredTransforms() {
        let natural = CGSize(width: 1_920, height: 1_080)
        let unsupported = [
            CGAffineTransform(a: 1.25, b: 0, c: 0, d: 0.8, tx: 0, ty: 0),
            CGAffineTransform(a: 1, b: 0, c: 0.25, d: 1, tx: 0, ty: 0),
            CGAffineTransform(a: 1, b: 1, c: 1, d: 1, tx: 0, ty: 0),
        ]

        for preferredTransform in unsupported {
            XCTAssertNil(CameraAnalysisRenderGeometryPolicy.make(
                naturalSize: natural,
                preferredTransform: preferredTransform,
                maximumPixelCount: 1_920 * 1_080,
                maximumDimensionPixels: 4_096,
                maximumAspectRatio: 4
            ), "encoded source rasters cannot smuggle scale, shear or collapse")
        }
    }

    func testRenderGeometryAcceptsQuantizedUnitRotationAndMirror() throws {
        let natural = CGSize(width: 1_920, height: 1_080)
        let quantizedQuarterTurn = CGAffineTransform(
            a: 0.000_05,
            b: 0.999_96,
            c: -1.000_03,
            d: 0.000_04,
            tx: 1_080,
            ty: 0
        )
        let rotation = try XCTUnwrap(CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: natural,
            preferredTransform: quantizedQuarterTurn,
            maximumPixelCount: 1_920 * 1_080,
            maximumDimensionPixels: 4_096,
            maximumAspectRatio: 4
        ))
        XCTAssertLessThanOrEqual(abs(rotation.pixelDimensions.x - 1_080), 2)
        XCTAssertLessThanOrEqual(abs(rotation.pixelDimensions.y - 1_920), 2)

        XCTAssertNotNil(CameraAnalysisRenderGeometryPolicy.make(
            naturalSize: natural,
            preferredTransform: CGAffineTransform(
                a: -0.999_96,
                b: 0.000_04,
                c: 0.000_05,
                d: 1.000_03,
                tx: 1_920,
                ty: 0
            ),
            maximumPixelCount: 1_920 * 1_080,
            maximumDimensionPixels: 4_096,
            maximumAspectRatio: 4
        ))
    }

    func testTextureEvidenceRejectsOneDimensionalStructure() throws {
        let size = 32
        let stripes = (0..<(size * size)).map { index in
            Double((index % size / 2).isMultiple(of: 2) ? 255 : 0)
        }
        let checker = (0..<(size * size)).map { index in
            let x = index % size
            let y = index / size
            return Double(((x / 2) + (y / 2)).isMultiple(of: 2) ? 255 : 0)
        }

        let stripeEvidence = try XCTUnwrap(CameraTextureQualityAnalyzer.analyze(
            luma: stripes,
            width: size,
            height: size
        ))
        let checkerEvidence = try XCTUnwrap(CameraTextureQualityAnalyzer.analyze(
            luma: checker,
            width: size,
            height: size
        ))

        XCTAssertLessThan(stripeEvidence.minimumStructureTensorEigenvalue, 1e-8)
        XCTAssertGreaterThan(checkerEvidence.minimumStructureTensorEigenvalue, 1e-4)
    }

    func testTextureEvidenceAcceptsDeterministicNonPeriodicTwoDimensionalTexture() throws {
        func mixed(_ input: UInt64) -> UInt64 {
            var value = input &+ 0x9E37_79B9_7F4A_7C15
            value = (value ^ (value >> 30)) &* 0xBF58_476D_1CE4_E5B9
            value = (value ^ (value >> 27)) &* 0x94D0_49BB_1331_11EB
            return value ^ (value >> 31)
        }
        let size = 64
        let luma = (0..<(size * size)).map {
            Double(mixed(UInt64($0)) & 0xff)
        }

        let evidence = try XCTUnwrap(CameraTextureQualityAnalyzer.analyze(
            luma: luma,
            width: size,
            height: size
        ))
        XCTAssertGreaterThan(evidence.minimumStructureTensorEigenvalue, 0.02)
    }

    func testFixedBoxAveragePlanUsesRenderPixelSpacing() throws {
        let plan = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
            imageSizePixels: SIMD2(80, 80),
            normalizedTile: CGRect(x: 0, y: 0, width: 1, height: 1),
            roundedPixelShift: SIMD2(3, -2),
            boxSizePixels: 8,
            boxSpacingPixels: 8
        ))

        XCTAssertEqual(plan.boxSizePixels, 8)
        XCTAssertEqual(plan.boxSpacingPixels, 8)
        XCTAssertEqual(plan.xOriginsPixels.first, 8)
        XCTAssertEqual(plan.xOriginsPixels.last, 72)
        XCTAssertEqual(plan.yOriginsPixels.first, 0)
        XCTAssertEqual(plan.yOriginsPixels.last, 64)
        XCTAssertEqual(plan.usablePixelArea, 72 * 72)
        XCTAssertEqual(
            plan.targetBoxOriginPixels(
                forReferenceBoxOriginPixels: SIMD2(8, 0)
            ),
            SIMD2(5, 2)
        )
        XCTAssertTrue(zip(plan.xOriginsPixels, plan.xOriginsPixels.dropFirst())
            .allSatisfy { $0.1 - $0.0 == 8 })
        XCTAssertTrue(zip(plan.yOriginsPixels, plan.yOriginsPixels.dropFirst())
            .allSatisfy { $0.1 - $0.0 == 8 })

        let negativeShift = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
            imageSizePixels: SIMD2(80, 80),
            normalizedTile: CGRect(x: 0, y: 0, width: 1, height: 1),
            roundedPixelShift: SIMD2(-3, 2),
            boxSizePixels: 8,
            boxSpacingPixels: 8
        ))
        XCTAssertEqual(negativeShift.xOriginsPixels.first, 0)
        XCTAssertEqual(negativeShift.yOriginsPixels.first, 8)
        XCTAssertEqual(
            negativeShift.targetBoxOriginPixels(
                forReferenceBoxOriginPixels: SIMD2(0, 8)
            ),
            SIMD2(3, 6)
        )
    }

    func testActualRegistrationPeaksRejectPeriod40ShiftedOnePeriod() throws {
        let spacing = 8
        let width = 48
        let height = 48
        let periodSamples = 40 / spacing
        let reference = (0..<(width * height)).map { index -> Double in
            let x = index % width
            let y = index / width
            return Double((x % periodSamples) * 37 + (y * y + 11 * y) % 251)
        }
        let target = (0..<(width * height)).map { index -> Double in
            let x = index % width
            let y = index / width
            let shiftedX = (x + periodSamples) % width
            return reference[y * width + shiftedX]
        }

        let evidence = try XCTUnwrap(CameraRegistrationPeakAnalyzer.analyze(
            referenceBoxAverages: reference,
            targetBoxAverages: target,
            width: width,
            height: height,
            renderOriginPixels: .zero,
            boxSpacingPixels: spacing,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48,
            minimumPeakSeparationPixels: 16,
            minimumBestCorrelation: 0.5
        ))

        XCTAssertEqual(evidence.bestOffsetPixels, .zero)
        XCTAssertEqual(evidence.bestCorrelation, 1, accuracy: 1e-12)
        XCTAssertEqual(evidence.secondBestCorrelation, 1, accuracy: 1e-12)
        XCTAssertLessThan(evidence.normalizedPeakSeparation, 0.05)
    }

    func testActualRegistrationPeaksAcceptBlurredNonPeriodicPair() throws {
        func mixed(_ input: UInt64) -> UInt64 {
            var value = input &+ 0x9E37_79B9_7F4A_7C15
            value = (value ^ (value >> 30)) &* 0xBF58_476D_1CE4_E5B9
            value = (value ^ (value >> 27)) &* 0x94D0_49BB_1331_11EB
            return value ^ (value >> 31)
        }
        let width = 48
        let height = 48
        let reference = (0..<(width * height)).map {
            Double(mixed(UInt64($0)) & 0xff)
        }
        var blurred = reference
        for y in 1..<(height - 1) {
            for x in 1..<(width - 1) {
                var sum = 0.0
                for dy in -1...1 {
                    for dx in -1...1 {
                        let weight = (dy == 0 ? 2.0 : 1.0)
                            * (dx == 0 ? 2.0 : 1.0)
                        sum += weight
                            * reference[(y + dy) * width + x + dx]
                    }
                }
                blurred[y * width + x] = sum / 16
            }
        }

        let evidence = try XCTUnwrap(CameraRegistrationPeakAnalyzer.analyze(
            referenceBoxAverages: reference,
            targetBoxAverages: blurred,
            width: width,
            height: height,
            renderOriginPixels: .zero,
            boxSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48,
            minimumPeakSeparationPixels: 16,
            minimumBestCorrelation: 0.5
        ))

        XCTAssertEqual(evidence.bestOffsetPixels, .zero)
        XCTAssertGreaterThan(evidence.bestCorrelation, 0.5)
        XCTAssertGreaterThan(evidence.normalizedPeakSeparation, 0.05)
        XCTAssertNil(CameraRegistrationPeakAnalyzer.analyze(
            referenceBoxAverages: reference,
            targetBoxAverages: blurred,
            width: width,
            height: height,
            renderOriginPixels: .zero,
            boxSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48,
            minimumPeakSeparationPixels: 16,
            minimumBestCorrelation: 0.9
        ), "a large relative peak gap cannot replace absolute pair correlation")
    }

    func testGlobalAliasScreenRejects384Period80WhenVisionClaimsZero() throws {
        let evidence = try peakEvidenceForPeriodicRenderShift(
            periodBoxSamples: SIMD2(10, 48),
            trueShiftBoxSamples: SIMD2(10, 0)
        )

        XCTAssertEqual(evidence.bestOffsetPixels, .zero)
        XCTAssertEqual(evidence.bestCorrelation, 1, accuracy: 1e-12)
        XCTAssertLessThan(
            evidence.normalizedPeakSeparation,
            0.05,
            "the global screen must see the true 80 px alias outside ±48 px"
        )
    }

    func testGlobalAliasScreenCoversEveryScaleRemainderAndSkinnyTail() throws {
        for period in [7, 10, 11, 17, 31, 40, 46] {
            let evidence = try peakEvidenceForPeriodicRenderShift(
                periodBoxSamples: SIMD2(period, 48),
                trueShiftBoxSamples: SIMD2(period, 0)
            )
            XCTAssertLessThan(
                evidence.normalizedPeakSeparation,
                0.05,
                "period \(period * 8) px must not fall between global scales"
            )
        }

        let diagonal = try peakEvidenceForPeriodicRenderShift(
            periodBoxSamples: SIMD2(11, 13),
            trueShiftBoxSamples: SIMD2(11, 13)
        )
        XCTAssertLessThan(
            diagonal.normalizedPeakSeparation,
            0.05,
            "the global screen must enumerate genuinely two-dimensional aliases"
        )
    }

    func testGlobalAliasPlanHasExactComplete384Budget() throws {
        let plan = try XCTUnwrap(
            CameraRegistrationPeakAnalyzer.makeCorrelationPlan(
                width: 48,
                height: 48,
                renderOriginPixels: .zero,
                boxSpacingPixels: 8,
                correlationSampleSpacingPixels: 32,
                maximumSearchRadiusPixels: 48
            )
        )

        XCTAssertEqual(plan.fineCandidateCount, 169)
        XCTAssertEqual(plan.finePairCountPerCandidate, 81)
        XCTAssertEqual(plan.broadAliasCandidateCount, 6_392)
        XCTAssertEqual(plan.tailAliasCandidateCount, 1_496)
        XCTAssertEqual(plan.zeroLagPrefixPairCount, 2_304)
        XCTAssertEqual(plan.totalCorrelationPairCount, 496_889)
        XCTAssertLessThanOrEqual(
            plan.totalCorrelationPairCount,
            CameraRegistrationPeakAnalyzer.maximumCorrelationPairCountPerTile
        )
    }

    func testGlobalAliasPlanRejectsOverflowAndUnboundedCandidateDomains() {
        XCTAssertNil(CameraRegistrationPeakAnalyzer.makeCorrelationPlan(
            width: Int.max,
            height: Int.max,
            renderOriginPixels: .zero,
            boxSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48
        ))
        XCTAssertNil(CameraRegistrationPeakAnalyzer.makeCorrelationPlan(
            width: 160,
            height: 90,
            renderOriginPixels: .zero,
            boxSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48
        ), "an over-budget tile must be subdivided rather than weakening K")
        XCTAssertNil(CameraRegistrationPeakAnalyzer.makeCorrelationPlan(
            width: 8,
            height: 13,
            renderOriginPixels: .zero,
            boxSpacingPixels: 8,
            correlationSampleSpacingPixels: 8,
            maximumSearchRadiusPixels: 8
        ), "fine offsets must be wholly contained by the broad alias domain")
    }

    func testEverySupportedTailOverlapContainsOneContiguous48PairDomain() throws {
        for width in 1...64 {
            for height in 1...64 where width * height >= 64 {
                let dimensions = try XCTUnwrap(
                    CameraRegistrationPeakAnalyzer.tailAliasDomainDimensions(
                        overlapWidth: width,
                        overlapHeight: height
                    ),
                    "\(width)x\(height)"
                )
                XCTAssertEqual(dimensions.x * dimensions.y, 48)
                XCTAssertLessThanOrEqual(dimensions.x, width)
                XCTAssertLessThanOrEqual(dimensions.y, height)
            }
        }
    }

    func testRegistrationPeakAnalyzerFailsClosedOnInvalidEvidence() {
        func mixed(_ input: UInt64) -> UInt64 {
            var value = input &+ 0x9E37_79B9_7F4A_7C15
            value = (value ^ (value >> 30)) &* 0xBF58_476D_1CE4_E5B9
            value = (value ^ (value >> 27)) &* 0x94D0_49BB_1331_11EB
            return value ^ (value >> 31)
        }
        func analyze(
            _ reference: [Double],
            _ target: [Double],
            width: Int = 48,
            height: Int = 48
        ) -> CameraRegistrationPeakEvidence? {
            CameraRegistrationPeakAnalyzer.analyze(
                referenceBoxAverages: reference,
                targetBoxAverages: target,
                width: width,
                height: height,
                renderOriginPixels: .zero,
                boxSpacingPixels: 8,
                correlationSampleSpacingPixels: 32,
                maximumSearchRadiusPixels: 48,
                minimumPeakSeparationPixels: 16,
                minimumBestCorrelation: 0.5
            )
        }

        let reference = (0..<(48 * 48)).map {
            Double(mixed(UInt64($0)) & 0xff)
        }
        var shiftedTarget = Array(repeating: 0.0, count: reference.count)
        for y in 0..<48 {
            for x in 1..<48 {
                shiftedTarget[y * 48 + x] = reference[y * 48 + x - 1]
            }
        }
        XCTAssertNil(analyze(reference, shiftedTarget),
            "Vision's zero candidate is not the actual global peak")

        var nonFinite = reference
        nonFinite[0] = .nan
        XCTAssertNil(analyze(nonFinite, reference))

        let flat = Array(repeating: 127.0, count: reference.count)
        XCTAssertNil(analyze(flat, flat))

        let small = (0..<(40 * 40)).map {
            Double(mixed(UInt64($0)) & 0xff)
        }
        XCTAssertNil(analyze(small, small, width: 40, height: 40),
            "the common inward domain has fewer than 64 paired samples")
        XCTAssertFalse(CameraRegistrationPeakAnalyzer
            .hasCompleteCandidateSupport(
                width: 1_920,
                height: 1_080,
                renderOriginPixels: .zero,
                boxSpacingPixels: 1,
                correlationSampleSpacingPixels: 1,
                maximumSearchRadiusPixels: 64
            ), "a relationally valid knob set cannot exceed the pair-cost cap")
    }

    func testRawRenderLumaFlowsThroughFixedBoxesIntoPeakEvidence() throws {
        func peak(
            reference: [Double],
            target: [Double],
            size: Int,
            claimedShift: SIMD2<Int>
        ) throws -> CameraRegistrationPeakEvidence {
            let plan = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
                imageSizePixels: SIMD2(repeating: size),
                normalizedTile: CGRect(x: 0, y: 0, width: 1, height: 1),
                roundedPixelShift: claimedShift,
                boxSizePixels: 8,
                boxSpacingPixels: 8
            ))
            let pair = try XCTUnwrap(CameraFixedBoxAveragePairSampler.sample(
                plan: plan,
                referenceLumaAt: { x, y in reference[y * size + x] },
                targetLumaAt: { x, y in target[y * size + x] }
            ))
            return try XCTUnwrap(CameraRegistrationPeakAnalyzer.analyze(
                referenceBoxAverages: pair.referenceBoxAverages,
                targetBoxAverages: pair.targetBoxAverages,
                width: pair.width,
                height: pair.height,
                renderOriginPixels: pair.renderOriginPixels,
                boxSpacingPixels: pair.boxSpacingPixels,
                correlationSampleSpacingPixels: 32,
                maximumSearchRadiusPixels: 48,
                minimumPeakSeparationPixels: 16,
                minimumBestCorrelation: 0.5
            ))
        }

        let size = 384
        let periodic = (0..<(size * size)).map { index -> Double in
            let x = index % size
            let y = index / size
            return Double((x % 40) * 5 + (y * y + 7 * y) % 251)
        }
        let shiftedPeriod = (0..<(size * size)).map { index -> Double in
            let x = index % size
            let y = index / size
            return periodic[y * size + (x + 40) % size]
        }
        let periodicPeak = try peak(
            reference: periodic,
            target: shiftedPeriod,
            size: size,
            claimedShift: SIMD2(40, 0)
        )
        XCTAssertLessThan(periodicPeak.normalizedPeakSeparation, 0.05)

        func mixed(_ input: UInt64) -> UInt64 {
            var value = input &+ 0x9E37_79B9_7F4A_7C15
            value = (value ^ (value >> 30)) &* 0xBF58_476D_1CE4_E5B9
            value = (value ^ (value >> 27)) &* 0x94D0_49BB_1331_11EB
            return value ^ (value >> 31)
        }
        let nonPeriodic = (0..<(size * size)).map {
            Double(mixed(UInt64($0)) & 0xff)
        }
        var blurred = nonPeriodic
        for y in 1..<(size - 1) {
            for x in 1..<(size - 1) {
                var sum = 0.0
                for dy in -1...1 {
                    for dx in -1...1 {
                        let weight = (dy == 0 ? 2.0 : 1.0)
                            * (dx == 0 ? 2.0 : 1.0)
                        sum += weight
                            * nonPeriodic[(y + dy) * size + x + dx]
                    }
                }
                blurred[y * size + x] = sum / 16
            }
        }
        let blurredPeak = try peak(
            reference: nonPeriodic,
            target: blurred,
            size: size,
            claimedShift: .zero
        )
        XCTAssertGreaterThan(blurredPeak.bestCorrelation, 0.5)
        XCTAssertGreaterThan(blurredPeak.normalizedPeakSeparation, 0.05)
    }

    private let fitConfiguration = CameraTileMotionFitter.Configuration(
        minimumTileCount: 3,
        minimumStructureTensorEigenvalue: 0.001,
        minimumRegistrationUniqueness: 0.1,
        maximumPostWarpMeanAbsoluteError: 20,
        maximumFitResidualPixels: 2,
        minimumSpatialEigenvalueFraction: 0.001
    )

    func testTileFitRecoversTranslationRotationAndScale() throws {
        let translation = SIMD2<Double>(7.25, -4.5)
        let rotation = 0.03
        let scale = 0.015
        let centres = [
            SIMD2<Double>(-160, -90), SIMD2<Double>(160, -90),
            SIMD2<Double>(-160, 90), SIMD2<Double>(160, 90),
        ]
        let registrations = centres.map { centre in
            CameraTileRegistration(
                centrePixels: centre,
                translationPixels: SIMD2(
                    translation.x - rotation * centre.y + scale * centre.x,
                    translation.y + rotation * centre.x + scale * centre.y
                ),
                areaFraction: 0.2,
                minimumStructureTensorEigenvalue: 0.01,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 2
            )
        }

        let fit = try XCTUnwrap(CameraTileMotionFitter.fit(
            registrations,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))

        XCTAssertEqual(fit.translationPixels.x, translation.x, accuracy: 1e-12)
        XCTAssertEqual(fit.translationPixels.y, translation.y, accuracy: 1e-12)
        XCTAssertEqual(fit.rotationRadians, rotation, accuracy: 1e-12)
        XCTAssertEqual(fit.scaleFraction, scale, accuracy: 1e-12)
        XCTAssertEqual(fit.fitResidualPixels, 0, accuracy: 1e-12)
        XCTAssertEqual(fit.normalizedTranslation.x, translation.x / 800, accuracy: 1e-12)
        XCTAssertGreaterThan(fit.registrationQuality, 0.5)
    }

    func testFlatBackgroundAndInconsistentTilesFailClosed() {
        let centres = [
            SIMD2<Double>(-160, -90), SIMD2<Double>(160, -90),
            SIMD2<Double>(-160, 90), SIMD2<Double>(160, 90),
        ]
        let flat = centres.map {
            CameraTileRegistration(
                centrePixels: $0,
                translationPixels: SIMD2(2, 1),
                areaFraction: 0.2,
                minimumStructureTensorEigenvalue: 0.000_001,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 0
            )
        }
        let inconsistent = zip(centres, [
            SIMD2<Double>(2, 1), SIMD2<Double>(-9, 7),
            SIMD2<Double>(11, -8), SIMD2<Double>(-5, -6),
        ]).map {
            CameraTileRegistration(
                centrePixels: $0.0,
                translationPixels: $0.1,
                areaFraction: 0.2,
                minimumStructureTensorEigenvalue: 0.01,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 2
            )
        }

        XCTAssertNil(CameraTileMotionFitter.fit(
            flat,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
        XCTAssertNil(CameraTileMotionFitter.fit(
            inconsistent,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
    }

    func testQualityValidMotionOutlierCannotBeDeletedToManufactureStaticFit() {
        let centres = [
            SIMD2<Double>(-160, -90), SIMD2<Double>(160, -90),
            SIMD2<Double>(-160, 90), SIMD2<Double>(160, 90),
        ]
        let translations = [
            SIMD2<Double>.zero, .zero, .zero, SIMD2<Double>(16, -12),
        ]
        let registrations = zip(centres, translations).map {
            CameraTileRegistration(
                centrePixels: $0.0,
                translationPixels: $0.1,
                areaFraction: 0.2,
                minimumStructureTensorEigenvalue: 0.01,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 1
            )
        }

        XCTAssertNil(CameraTileMotionFitter.fit(
            registrations,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
    }

    func testTooFewOrDegenerateTileCentresCannotFitRotation() {
        let one = CameraTileRegistration(
            centrePixels: .zero,
            translationPixels: .zero,
            areaFraction: 0.2,
            minimumStructureTensorEigenvalue: 0.01,
            registrationUniqueness: 0.8,
            postWarpMeanAbsoluteError: 0
        )
        XCTAssertNil(CameraTileMotionFitter.fit(
            [one, one],
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
        XCTAssertNil(CameraTileMotionFitter.fit(
            [one, one, one],
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
    }

    func testOnlyQualityValidTilesContributeToUsableBackground() throws {
        let translation = SIMD2<Double>(3, -2)
        let centres = [
            SIMD2<Double>(-100, -100),
            SIMD2<Double>(100, -100),
            SIMD2<Double>(0, 100),
        ]
        var registrations = centres.map {
            CameraTileRegistration(
                centrePixels: $0,
                translationPixels: translation,
                areaFraction: 0.1,
                minimumStructureTensorEigenvalue: 0.01,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 1
            )
        }
        registrations.append(CameraTileRegistration(
            centrePixels: .zero,
            translationPixels: SIMD2(500, -500),
            areaFraction: 0.6,
            minimumStructureTensorEigenvalue: 0.000_001,
            registrationUniqueness: 0,
            postWarpMeanAbsoluteError: 100
        ))

        let fit = try XCTUnwrap(CameraTileMotionFitter.fit(
            registrations,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
        XCTAssertEqual(fit.translationPixels.x, translation.x, accuracy: 1e-12)
        XCTAssertEqual(fit.translationPixels.y, translation.y, accuracy: 1e-12)
        XCTAssertEqual(fit.usableBackgroundFraction, 0.3, accuracy: 1e-12)
        XCTAssertEqual(fit.usedTileCount, 3)
    }

    func testCollinearQualityTilesFailSpatialSpreadGate() {
        let registrations = [-100.0, 0, 100].map {
            CameraTileRegistration(
                centrePixels: SIMD2($0, 0),
                translationPixels: SIMD2(2, 1),
                areaFraction: 0.2,
                minimumStructureTensorEigenvalue: 0.01,
                registrationUniqueness: 0.8,
                postWarpMeanAbsoluteError: 1
            )
        }

        XCTAssertNil(CameraTileMotionFitter.fit(
            registrations,
            imageDiagonalPixels: 800,
            configuration: fitConfiguration
        ))
    }

    func testEachMetricSelectsItsOwnStrongestAnchor() throws {
        func estimate(
            baseline: TimeInterval,
            translation: Double,
            rotation: Double,
            scale: Double
        ) -> CameraAnchoredMotionEstimate {
            CameraAnchoredMotionEstimate(
                baselineTimestamp: baseline,
                fit: CameraTileMotionFit(
                    translationPixels: SIMD2(translation * 800, 0),
                    normalizedTranslation: SIMD2(translation, 0),
                    rotationRadians: rotation,
                    scaleFraction: scale,
                    fitResidualPixels: 0,
                    registrationQuality: 0.9,
                    usableBackgroundFraction: 0.3,
                    usedTileCount: 3
                )
            )
        }
        let peaks = try XCTUnwrap(CameraAnchorPeakSelector.select([
            estimate(baseline: 0.1, translation: 0.03, rotation: 0.001, scale: 0.001),
            estimate(baseline: 0.2, translation: 0.01, rotation: 0.04, scale: 0.002),
            estimate(baseline: 0.3, translation: 0.005, rotation: 0.003, scale: 0.05),
        ]))

        XCTAssertEqual(peaks.translation.baselineTimestamp, 0.1)
        XCTAssertEqual(peaks.rotation.baselineTimestamp, 0.2)
        XCTAssertEqual(peaks.scale.baselineTimestamp, 0.3)
    }

    func testBackgroundTilesAreTheNonOverlappingComplementOfInflatedPersonUnion() {
        let first = CGRect(x: 0.38, y: 0.12, width: 0.20, height: 0.70)
        let second = CGRect(x: 0.42, y: 0.10, width: 0.22, height: 0.72)
        let plan = CameraBackgroundRegionPlanner.make(
            personBoxes: [first, second],
            inflationFraction: 0.03,
            minimumTileAreaFraction: 0.01
        )

        XCTAssertEqual(plan.excludedPersonRegion,
                       CGRect(x: 0.35, y: 0.07, width: 0.32, height: 0.78))
        XCTAssertEqual(plan.backgroundFraction,
                       1 - plan.excludedPersonRegion.width * plan.excludedPersonRegion.height,
                       accuracy: 1e-12)
        XCTAssertGreaterThanOrEqual(plan.tiles.count, 3)
        for tile in plan.tiles {
            XCTAssertTrue(CGRect(x: 0, y: 0, width: 1, height: 1).contains(tile))
            XCTAssertFalse(tile.intersects(plan.excludedPersonRegion))
        }
        for left in plan.tiles.indices {
            for right in plan.tiles.indices where left < right {
                XCTAssertTrue(plan.tiles[left].intersection(plan.tiles[right]).isNull
                              || plan.tiles[left].intersection(plan.tiles[right]).isEmpty)
            }
        }
    }

    func testPersonCoveringMostOfFrameLeavesNoUsableBackgroundTiles() {
        let plan = CameraBackgroundRegionPlanner.make(
            personBoxes: [CGRect(x: 0.02, y: 0.02, width: 0.96, height: 0.96)],
            inflationFraction: 0.03,
            minimumTileAreaFraction: 0.01
        )
        XCTAssertEqual(plan.backgroundFraction, 0)
        XCTAssertTrue(plan.tiles.isEmpty)
    }

    func testPeakFeasibilityFiltersNarrowStripsAndSplitsUsableSideTiles() throws {
        let frameSize = SIMD2(1_920, 1_080)
        let plan = try XCTUnwrap(CameraBackgroundAnalysisTilePlanner.make(
            backgroundTiles: [
                CGRect(x: 0, y: 0, width: 0.2, height: 1),
                CGRect(x: 0.8, y: 0, width: 0.2, height: 1),
                CGRect(x: 0.2, y: 0, width: 0.6, height: 0.07),
                CGRect(x: 0.2, y: 0.93, width: 0.6, height: 0.07),
            ],
            imageSizePixels: frameSize,
            minimumTileAreaFraction: 0.01,
            boxAverageSizePixels: 8,
            boxAverageSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            peakSearchRadiusPixels: 48
        ))

        XCTAssertEqual(plan.tiles.count, 6)
        XCTAssertGreaterThan(plan.sampledBackgroundFraction, 0.35)
        XCTAssertLessThanOrEqual(plan.sampledBackgroundFraction, 0.4)
        for tile in plan.tiles {
            let grid = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
                imageSizePixels: frameSize,
                normalizedTile: tile,
                roundedPixelShift: .zero,
                boxSizePixels: 8,
                boxSpacingPixels: 8
            ))
            XCTAssertTrue(CameraRegistrationPeakAnalyzer
                .hasCompleteCandidateSupport(
                    width: grid.xOriginsPixels.count,
                    height: grid.yOriginsPixels.count,
                    renderOriginPixels: SIMD2(
                        try XCTUnwrap(grid.xOriginsPixels.first),
                        try XCTUnwrap(grid.yOriginsPixels.first)
                    ),
                    boxSpacingPixels: 8,
                    correlationSampleSpacingPixels: 32,
                    maximumSearchRadiusPixels: 48
                ))
        }
    }

    func testPeakFeasibilityRecursivelyPartitionsCanonicalCalibratedRaster() throws {
        let plan = try XCTUnwrap(CameraBackgroundAnalysisTilePlanner.make(
            backgroundTiles: [CGRect(x: 0, y: 0, width: 1, height: 1)],
            imageSizePixels: SIMD2(1_280, 720),
            minimumTileAreaFraction: 0.01,
            boxAverageSizePixels: 8,
            boxAverageSpacingPixels: 8,
            correlationSampleSpacingPixels: 32,
            peakSearchRadiusPixels: 48
        ))

        XCTAssertLessThanOrEqual(
            plan.tiles.count,
            CameraBackgroundAnalysisTilePlanner.maximumAnalysisTileCount
        )
        XCTAssertGreaterThanOrEqual(plan.tiles.count, 3)
        for tile in plan.tiles {
            let grid = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
                imageSizePixels: SIMD2(1_280, 720),
                normalizedTile: tile,
                roundedPixelShift: .zero,
                boxSizePixels: 8,
                boxSpacingPixels: 8
            ))
            XCTAssertTrue(CameraRegistrationPeakAnalyzer
                .hasCompleteCandidateSupport(
                    width: grid.xOriginsPixels.count,
                    height: grid.yOriginsPixels.count,
                    renderOriginPixels: SIMD2(
                        try XCTUnwrap(grid.xOriginsPixels.first),
                        try XCTUnwrap(grid.yOriginsPixels.first)
                    ),
                    boxSpacingPixels: 8,
                    correlationSampleSpacingPixels: 32,
                    maximumSearchRadiusPixels: 48
                ))
        }
    }

    func testVisionTranslationValidationRejectsNonFiniteAndNonTranslationTransforms() throws {
        let size = SIMD2<Int>(1_280, 720)
        let valid = try XCTUnwrap(
            CameraVisionTranslationPolicy.validate(
                CGAffineTransform(translationX: 3.4, y: -2.6),
                imageSizePixels: size
            )
        )
        XCTAssertEqual(valid.translationPixels, SIMD2(3.4, -2.6))
        XCTAssertEqual(valid.roundedPixelShift, SIMD2(3, -3))

        XCTAssertNil(CameraVisionTranslationPolicy.validate(
            CGAffineTransform(a: 1, b: 0, c: 0, d: 1, tx: .nan, ty: 0),
            imageSizePixels: size
        ))
        XCTAssertNil(CameraVisionTranslationPolicy.validate(
            CGAffineTransform(rotationAngle: 0.01),
            imageSizePixels: size
        ))
        XCTAssertNil(CameraVisionTranslationPolicy.validate(
            CGAffineTransform(scaleX: 1.01, y: 1),
            imageSizePixels: size
        ))
        XCTAssertNil(CameraVisionTranslationPolicy.validate(
            CGAffineTransform(translationX: 1_281, y: 0),
            imageSizePixels: size
        ))
    }

    func testEveryPlannedVisionTileMustProduceExactlyOneValidatedRegistration() {
        XCTAssertTrue(CameraPlannedTileEvidencePolicy.isComplete(
            plannedTileCount: 4,
            validatedRegistrationCount: 4
        ))
        XCTAssertFalse(CameraPlannedTileEvidencePolicy.isComplete(
            plannedTileCount: 4,
            validatedRegistrationCount: 3
        ))
        XCTAssertFalse(CameraPlannedTileEvidencePolicy.isComplete(
            plannedTileCount: 4,
            validatedRegistrationCount: 5
        ))
        XCTAssertFalse(CameraPlannedTileEvidencePolicy.isComplete(
            plannedTileCount: 0,
            validatedRegistrationCount: 0
        ))
    }

    private func peakEvidenceForPeriodicRenderShift(
        periodBoxSamples: SIMD2<Int>,
        trueShiftBoxSamples: SIMD2<Int>
    ) throws -> CameraRegistrationPeakEvidence {
        let renderSize = 384
        let boxSize = 8
        func mixed(_ input: UInt64) -> UInt64 {
            var value = input &+ 0x9E37_79B9_7F4A_7C15
            value = (value ^ (value >> 30)) &* 0xBF58_476D_1CE4_E5B9
            value = (value ^ (value >> 27)) &* 0x94D0_49BB_1331_11EB
            return value ^ (value >> 31)
        }
        func motif(x: Int, y: Int) -> Double {
            let boxX = x / boxSize
            let boxY = y / boxSize
            let phaseX = boxX % periodBoxSamples.x
            let phaseY = boxY % periodBoxSamples.y
            return Double(mixed(UInt64(phaseY * periodBoxSamples.x + phaseX)) & 0xff)
        }

        let plan = try XCTUnwrap(CameraFixedBoxAverageGridPolicy.make(
            imageSizePixels: SIMD2(repeating: renderSize),
            normalizedTile: CGRect(x: 0, y: 0, width: 1, height: 1),
            roundedPixelShift: .zero,
            boxSizePixels: boxSize,
            boxSpacingPixels: boxSize
        ))
        let pair = try XCTUnwrap(CameraFixedBoxAveragePairSampler.sample(
            plan: plan,
            referenceLumaAt: { x, y in motif(x: x, y: y) },
            targetLumaAt: { x, y in
                motif(
                    x: x + trueShiftBoxSamples.x * boxSize,
                    y: y + trueShiftBoxSamples.y * boxSize
                )
            }
        ))
        return try XCTUnwrap(CameraRegistrationPeakAnalyzer.analyze(
            referenceBoxAverages: pair.referenceBoxAverages,
            targetBoxAverages: pair.targetBoxAverages,
            width: pair.width,
            height: pair.height,
            renderOriginPixels: pair.renderOriginPixels,
            boxSpacingPixels: pair.boxSpacingPixels,
            correlationSampleSpacingPixels: 32,
            maximumSearchRadiusPixels: 48,
            minimumPeakSeparationPixels: 16,
            minimumBestCorrelation: 0.5
        ))
    }
}
