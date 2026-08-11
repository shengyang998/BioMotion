import CoreML
import simd
import XCTest
@testable import BioMotion

/// Covers the camera-relative root-position transform the offline solver input
/// omits, plus the position-variation diagnostic. Neither is dynamics authority.
///
/// # The finding these tests encode
/// `joint_coords` zeroes `global_trans`, so the pelvis is pinned at the model
/// constant (0, 0.924, 0) in every frame. The model emits camera-relative root
/// position separately as `cam_t`; the app exports it, stores it on
/// `FrameResult`, and uses it to project the overlay.
/// `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` can compose that position for
/// geometry/projection tests, while tagging the result position-only.
///
/// Verified on 309 frames of `video_012.mov` through the shipping Core ML model
/// (`labs/sam-3d-body/export/camt_probe.py`, 2026-08-07): depth 4.34 m, 1.10 m
/// below the optical axis, ±0.37 m lateral, `corr(1/bbox_side, depth) = +0.74`
/// exactly as `camera_head.py`'s `tz = 2f/(bbox_side·s)` requires.
///
/// ⚠️ Necessary, not sufficient — see `MHRRetarget`'s header for the measured
/// depth noise and STATUS.md for what it rules out.
final class RootTranslationTests: XCTestCase {

    private enum Idx {
        static let root = 1, lUpleg = 2, lLowleg = 3, lFoot = 4, lBall = 8
        static let rUpleg = 18, rLowleg = 19, rFoot = 20, rBall = 24
        static let cSpine1 = 35, cSpine2 = 36, cSpine3 = 37
        static let rUparm = 39, rLowarm = 40, rWrist = 42
        static let lUparm = 75, lLowarm = 76, lWrist = 78
        static let cNeck = 110, cHead = 113, cHeadNull = 126
        static let count = 127
    }

    /// A straight upright MHR skeleton with the pelvis at the model constant —
    /// i.e. shaped like a real `joint_coords` output, pinning included.
    private func pinnedSkeleton(shiftedBy d: SIMD3<Float> = .zero) -> [SIMD3<Float>] {
        var j = [SIMD3<Float>](repeating: SIMD3<Float>(0, 0.924, 0), count: Idx.count)
        func set(_ i: Int, _ x: Float, _ y: Float, _ z: Float) {
            j[i] = SIMD3<Float>(x, y + 0.924, z) + d
        }
        set(Idx.root, 0, 0, 0)
        set(Idx.lUpleg, -0.085, 0, 0);  set(Idx.rUpleg, 0.085, 0, 0)
        set(Idx.lLowleg, -0.085, -0.42, 0); set(Idx.rLowleg, 0.085, -0.42, 0)
        set(Idx.lFoot, -0.085, -0.84, 0);   set(Idx.rFoot, 0.085, -0.84, 0)
        set(Idx.lBall, -0.085, -0.89, 0.12); set(Idx.rBall, 0.085, -0.89, 0.12)
        set(Idx.cSpine1, 0, 0.13, 0); set(Idx.cSpine2, 0, 0.24, 0); set(Idx.cSpine3, 0, 0.42, 0)
        set(Idx.cNeck, 0, 0.52, 0); set(Idx.cHead, 0, 0.61, 0); set(Idx.cHeadNull, 0, 0.80, 0)
        set(Idx.lUparm, -0.17, 0.46, 0);  set(Idx.rUparm, 0.17, 0.46, 0)
        set(Idx.lLowarm, -0.17, 0.17, 0); set(Idx.rLowarm, 0.17, 0.17, 0)
        set(Idx.lWrist, -0.17, -0.09, 0); set(Idx.rWrist, 0.17, -0.09, 0)
        // Everything not listed keeps the pinned constant, which is what a real
        // `joint_coords` does for joints this app never reads.
        return j
    }

    private func positions(_ f: BodyFrame) -> [String: SIMD3<Float>] {
        Dictionary(uniqueKeysWithValues: f.joints.map { ($0.id, $0.worldPosition) })
    }

    private static func outputProvider(
        camT: SIMD3<Float>,
        camTShape: [Int] = [3],
        invalidJoint: (index: Int, component: Int, value: Float)? = nil,
        rotationElement: (index: Int, row: Int, column: Int, value: Float)? = nil,
        keypoint: (index: Int, value: SIMD2<Float>)? = nil,
        dataTypes: [String: MLMultiArrayDataType] = [:]
    ) throws -> MLDictionaryFeatureProvider {
        func array(_ shape: [Int], feature: String) throws -> MLMultiArray {
            let value = try MLMultiArray(
                shape: shape.map(NSNumber.init(value:)),
                dataType: dataTypes[feature] ?? .float32
            )
            for index in 0..<value.count { value[index] = 0 }
            return value
        }

        let jointCount = SAM3DPoseEstimator.PreprocessingConstants.numBodyJoints
        let joints = try array([jointCount, 3], feature: "joint_coords")
        if let invalidJoint {
            joints[invalidJoint.index * 3 + invalidJoint.component] =
                NSNumber(value: invalidJoint.value)
        }
        let rotations = try array([jointCount, 3, 3], feature: "global_rots")
        for joint in 0..<jointCount {
            for axis in 0..<3 {
                rotations[joint * 9 + axis * 3 + axis] = 1
            }
        }
        if let rotationElement {
            rotations[
                rotationElement.index * 9
                    + rotationElement.row * 3
                    + rotationElement.column
            ] = NSNumber(value: rotationElement.value)
        }
        let camera = try array(camTShape, feature: "cam_t")
        camera[0] = NSNumber(value: camT.x)
        camera[1] = NSNumber(value: camT.y)
        camera[2] = NSNumber(value: camT.z)
        let keypoints = try array(
            [SAM3DPoseEstimator.PreprocessingConstants.numOutputKeypoints2D, 2],
            feature: "keypoints_2d"
        )
        if let keypoint {
            keypoints[keypoint.index * 2] = NSNumber(value: keypoint.value.x)
            keypoints[keypoint.index * 2 + 1] = NSNumber(value: keypoint.value.y)
        }
        return try MLDictionaryFeatureProvider(dictionary: [
            "joint_coords": MLFeatureValue(multiArray: joints),
            "global_rots": MLFeatureValue(multiArray: rotations),
            "cam_t": MLFeatureValue(multiArray: camera),
            "keypoints_2d": MLFeatureValue(multiArray: keypoints),
        ])
    }

    // MARK: - The default is byte-for-byte the old behaviour

    /// The parameter is opt-in. Every existing caller omits it and must get the
    /// pelvis-pinned frame it got before the parameter existed — otherwise this
    /// change would silently alter the live ARKit path's sibling and every
    /// fixture in the suite.
    func testOmittingCamTIsIdenticalToPassingNil() {
        let j = pinnedSkeleton()
        let a = MHRRetarget.makeBodyFrame(jointCoords: j, timestamp: 1.0, frameNumber: 3)
        let b = MHRRetarget.makeBodyFrame(jointCoords: j, camT: nil, timestamp: 1.0, frameNumber: 3)
        XCTAssertEqual(a.dynamicsReference, .mhrRootRelative)
        XCTAssertEqual(b.dynamicsReference, .mhrRootRelative)
        XCTAssertEqual(a.joints.count, b.joints.count)
        for (index, pair) in zip(a.joints, b.joints).enumerated() {
            let (x, y) = pair
            XCTAssertEqual(x.id, y.id)
            XCTAssertEqual(x.worldPosition, y.worldPosition,
                           "the default must be bit-identical to the pre-existing behaviour")
            XCTAssertEqual(x.opensimMarkerNameOverride, MHRRetarget.table[index].opensimMarker)
            XCTAssertEqual(y.opensimMarkerNameOverride, MHRRetarget.table[index].opensimMarker)
        }
        XCTAssertEqual(a.joints.first?.opensimMarkerNameOverride, "MHR_ROOT")
        XCTAssertTrue(MHRRetarget.inconsistenciesWithJointMapping().isEmpty,
                      MHRRetarget.inconsistenciesWithJointMapping().joined(separator: "; "))
    }

    // MARK: - What composing cam_t does, and does not, change

    /// The composition is a pure translation: every inter-marker distance —
    /// i.e. the relative geometry IK fits — is unchanged to float precision.
    /// This proves geometry preservation only and does not authorize production
    /// activation: camera motion, gravity and root/depth derivative quality
    /// require independent evidence.
    func testCamTIsARigidTranslationAndChangesNoRelativeGeometry() {
        let j = pinnedSkeleton()
        let camT = SIMD3<Float>(0.31, 1.10, 4.34)      // the measured scale on real video
        let pinnedFrame = MHRRetarget.makeBodyFrame(jointCoords: j, timestamp: 0, frameNumber: 0)
        let cameraFrame = MHRRetarget.makeBodyFrame(jointCoords: j, camT: camT,
                                                    timestamp: 0, frameNumber: 0)
        let pinned = positions(pinnedFrame)
        let world = positions(cameraFrame)
        let expected = MHRRetarget.rootTranslation(camT: camT)

        XCTAssertEqual(pinnedFrame.dynamicsReference, .mhrRootRelative)
        XCTAssertEqual(cameraFrame.dynamicsReference, .mhrCameraRelativePosition)
        XCTAssertFalse(cameraFrame.dynamicsReference.permits(.staticEquilibrium))
        XCTAssertFalse(cameraFrame.dynamicsReference.permits(.temporal))

        var maxOffsetError: Float = 0
        var maxDistanceError: Float = 0
        let ids = pinned.keys.sorted()
        for id in ids {
            maxOffsetError = max(maxOffsetError,
                                 simd_length((world[id]! - pinned[id]!) - expected))
        }
        for a in ids {
            for b in ids where a < b {
                let dp = simd_length(pinned[a]! - pinned[b]!)
                let dw = simd_length(world[a]! - world[b]!)
                maxDistanceError = max(maxDistanceError, abs(dp - dw))
            }
        }
        print("ROOT-METRIC translation=\(expected) max_offset_err=\(maxOffsetError) "
            + "max_pairwise_distance_change=\(maxDistanceError)")

        XCTAssertLessThan(maxOffsetError, 1e-5, "every marker must move by exactly the root translation")
        XCTAssertLessThan(maxDistanceError, 1e-4,
                          "relative geometry — everything IK fits — must be untouched")
    }

    /// Ties the new composition to the ONE piece of this geometry that has been
    /// validated against ground truth: the model's own projection.
    ///
    /// `projectToImage` was measured at 1.0 px mean (0.0 px at the shoulders)
    /// against the model's own `keypoints_2d` (`projection_selfcheck.py`). So
    /// if projecting a `camT`-composed marker through a ZERO `cam_t` lands on
    /// the same pixel as projecting the pinned marker through the real `cam_t`,
    /// the composition inherits that validation instead of asserting a new
    /// convention. It is the same statement as "the axis flip is undone once,
    /// consistently", written as pixels.
    func testComposedMarkersProjectToTheSamePixelsAsTheOldPath() {
        let j = pinnedSkeleton()
        let camT = SIMD3<Float>(-0.22, 0.94, 3.80)
        let size = CGSize(width: 576, height: 1024)
        let pinned = positions(MHRRetarget.makeBodyFrame(jointCoords: j, timestamp: 0, frameNumber: 0))
        let world = positions(MHRRetarget.makeBodyFrame(jointCoords: j, camT: camT,
                                                        timestamp: 0, frameNumber: 0))
        let pinnedBody = MHRRetarget.makeBodyFrame(
            jointCoords: j, timestamp: 0, frameNumber: 0)
        let composedBody = MHRRetarget.makeBodyFrame(
            jointCoords: j, camT: camT, timestamp: 0, frameNumber: 0)

        XCTAssertEqual(PhotoOverlayView.projectionCameraTranslation(
            bodyFrame: pinnedBody, storedCameraTranslation: camT), camT)
        XCTAssertEqual(PhotoOverlayView.projectionCameraTranslation(
            bodyFrame: composedBody, storedCameraTranslation: camT), .zero,
                       "a camera-relative body already contains cam_t")
        XCTAssertNil(PhotoOverlayView.projectionCameraTranslation(
            bodyFrame: BodyFrame(timestamp: 0, frameNumber: 0, joints: []),
            storedCameraTranslation: camT),
                     "an untyped coordinate frame must not be projected by assumption")
        for inconsistentReference in [
            BodyFrame.DynamicsReference(
                gravity: .gravityAligned,
                rootTrajectory: .rootRelative
            ),
            BodyFrame.DynamicsReference(
                gravity: .gravityAligned,
                rootTrajectory: .cameraRelativePositionOnly
            ),
        ] {
            XCTAssertNil(PhotoOverlayView.projectionCameraTranslation(
                bodyFrame: BodyFrame(
                    timestamp: 0,
                    frameNumber: 0,
                    joints: [],
                    dynamicsReference: inconsistentReference
                ),
                storedCameraTranslation: camT
            ), "a root enum case alone does not prove this frame matches stored MHR cam_t")
        }

        var maxPixelGap: CGFloat = 0
        var maxDoubleTranslationGap: CGFloat = 0
        var compared = 0
        for id in pinned.keys.sorted() {
            guard let old = MHRRetarget.projectToImage(pinned[id]!, camT: camT, imageSize: size),
                  let new = MHRRetarget.projectToImage(world[id]!, camT: .zero, imageSize: size)
            else { continue }
            maxPixelGap = max(maxPixelGap, hypot(old.x - new.x, old.y - new.y))
            if let doubled = MHRRetarget.projectToImage(
                world[id]!, camT: camT, imageSize: size
            ) {
                maxDoubleTranslationGap = max(
                    maxDoubleTranslationGap,
                    hypot(old.x - doubled.x, old.y - doubled.y)
                )
            }
            compared += 1
        }
        print("ROOT-METRIC projection compared=\(compared) max_pixel_gap=\(maxPixelGap)")
        XCTAssertEqual(compared, 20, "all twenty markers must project in front of the camera")
        XCTAssertLessThan(maxPixelGap, 0.5,
                          "composing cam_t must be the same transform the validated projection applies")
        XCTAssertGreaterThan(maxDoubleTranslationGap, 20,
                             "a composed marker plus raw cam_t applies translation twice")
    }

    /// The depth sign is the one that is easy to get backwards and expensive to
    /// get wrong (it would put the subject behind the camera). `cam_t.z` is a
    /// POSITIVE distance in front of the camera in the OpenCV-style frame, so
    /// in this file's Y-up frame — camera at the origin looking along −Z — the
    /// subject must land at NEGATIVE z, and BELOW the optical axis for a phone
    /// held above hip height.
    func testRootTranslationSigns() throws {
        let t = MHRRetarget.rootTranslation(camT: SIMD3<Float>(0.31, 1.10, 4.34))
        print("ROOT-METRIC signs \(t)")
        XCTAssertEqual(t.x, 0.31, accuracy: 1e-6, "lateral passes through unchanged")
        XCTAssertEqual(t.y, -1.10, accuracy: 1e-6, "cam_t.y is DOWN-positive; the Y-up frame negates it")
        XCTAssertEqual(t.z, -4.34, accuracy: 1e-6, "cam_t.z is AWAY-positive; −Z is in front of the camera")

        XCTAssertTrue(MHRRetarget.isValidCameraTranslation(SIMD3<Float>(0, 0, 0.01)))
        for invalid in [
            SIMD3<Float>(.nan, 0, 1),
            SIMD3<Float>(0, .infinity, 1),
            SIMD3<Float>(0, 0, 0),
            SIMD3<Float>(0, 0, -1),
            SIMD3<Float>(1_001, 0, 4),
            SIMD3<Float>(0, 0, 1_001),
        ] {
            XCTAssertFalse(MHRRetarget.isValidCameraTranslation(invalid), "\(invalid)")
            XCTAssertTrue(MHRRetarget.makeBodyFrame(
                jointCoords: pinnedSkeleton(), camT: invalid,
                timestamp: 0, frameNumber: 0
            ).joints.isEmpty, "invalid cam_t must fail closed before IK")

            let provider = try Self.outputProvider(camT: invalid)
            XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
                provider, usedFallbackBBox: false, inputChecksum: 0,
                sourceHash: 0, bboxHash: 0, warpHash: 0
            )) { error in
                guard case SAM3DPoseEstimator.EstimatorError.invalidOutputValue(let detail) = error
                else { return XCTFail("wrong parser error: \(error)") }
                XCTAssertTrue(detail.contains("cam_t"), detail)
            }
        }

        let valid = try SAM3DPoseEstimator.parseOutput(
            Self.outputProvider(camT: SIMD3<Float>(0, 1, 4)),
            usedFallbackBBox: false, inputChecksum: 0,
            sourceHash: 0, bboxHash: 0, warpHash: 0
        )
        XCTAssertEqual(valid.camT, SIMD3<Float>(0, 1, 4))

        let wrongRankCamera = try Self.outputProvider(
            camT: SIMD3<Float>(0, 1, 4),
            camTShape: [1, 3]
        )
        XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
            wrongRankCamera, usedFallbackBBox: false, inputChecksum: 0,
            sourceHash: 0, bboxHash: 0, warpHash: 0
        )) { error in
            guard case SAM3DPoseEstimator.EstimatorError
                .unexpectedOutputShape(let detail) = error
            else { return XCTFail("wrong parser error: \(error)") }
            XCTAssertTrue(detail.contains("cam_t: expected [3]"), detail)
        }

        let invalidJoint = try Self.outputProvider(
            camT: SIMD3<Float>(0, 1, 4),
            invalidJoint: (Idx.cSpine1, 0, .nan)
        )
        XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
            invalidJoint, usedFallbackBBox: false, inputChecksum: 0,
            sourceHash: 0, bboxHash: 0, warpHash: 0
        )) { error in
            guard case SAM3DPoseEstimator.EstimatorError.invalidOutputValue(let detail) = error
            else { return XCTFail("wrong parser error: \(error)") }
            XCTAssertTrue(detail.contains("joint_coords[\(Idx.cSpine1)]"), detail)
        }

        var directJointCoords = pinnedSkeleton()
        directJointCoords[Idx.cSpine1].x = .infinity
        XCTAssertTrue(MHRRetarget.makeBodyFrame(
            jointCoords: directJointCoords, timestamp: 0, frameNumber: 0
        ).joints.isEmpty, "direct retarget callers must also fail closed")
    }

    func testParserRequiresFloat32ForEveryOutputFeature() throws {
        let features = ["joint_coords", "global_rots", "cam_t", "keypoints_2d"]
        for feature in features {
            for dataType: MLMultiArrayDataType in [.float16, .double] {
                let provider = try Self.outputProvider(
                    camT: SIMD3<Float>(0, 1, 4),
                    dataTypes: [feature: dataType]
                )
                XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
                    provider, usedFallbackBBox: false, inputChecksum: 0,
                    sourceHash: 0, bboxHash: 0, warpHash: 0
                ), "\(feature) must reject \(dataType)") { error in
                    guard case SAM3DPoseEstimator.EstimatorError
                        .unexpectedOutputDataType(let detail) = error
                    else { return XCTFail("wrong parser error for \(feature): \(error)") }
                    XCTAssertTrue(detail.contains(feature), detail)
                    XCTAssertTrue(detail.contains("Float32"), detail)
                }
            }
        }
    }

    func testParserRejectsNonFiniteGlobalRotations() throws {
        let invalidElements: [(row: Int, column: Int, value: Float)] = [
            (0, 1, .nan),
            (1, 2, .infinity),
            (2, 0, -.infinity),
        ]
        for invalid in invalidElements {
            let provider = try Self.outputProvider(
                camT: SIMD3<Float>(0, 1, 4),
                rotationElement: (7, invalid.row, invalid.column, invalid.value)
            )
            XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
                provider, usedFallbackBBox: false, inputChecksum: 0,
                sourceHash: 0, bboxHash: 0, warpHash: 0
            )) { error in
                guard case SAM3DPoseEstimator.EstimatorError.invalidOutputValue(let detail) = error
                else { return XCTFail("wrong parser error: \(error)") }
                XCTAssertTrue(
                    detail.contains("global_rots[7][\(invalid.row)][\(invalid.column)]"),
                    detail
                )
            }
        }
    }

    func testParserRejectsNonFiniteKeypointsAndPreservesFiniteContractValues() throws {
        for invalid in [
            SIMD2<Float>(.nan, 12),
            SIMD2<Float>(12, .infinity),
            SIMD2<Float>(-.infinity, 12),
        ] {
            let provider = try Self.outputProvider(
                camT: SIMD3<Float>(0, 1, 4),
                keypoint: (11, invalid)
            )
            XCTAssertThrowsError(try SAM3DPoseEstimator.parseOutput(
                provider, usedFallbackBBox: false, inputChecksum: 0,
                sourceHash: 0, bboxHash: 0, warpHash: 0
            )) { error in
                guard case SAM3DPoseEstimator.EstimatorError.invalidOutputValue(let detail) = error
                else { return XCTFail("wrong parser error: \(error)") }
                XCTAssertTrue(detail.contains("keypoints_2d[11]"), detail)
            }
        }

        let finiteButUnconstrained = try SAM3DPoseEstimator.parseOutput(
            Self.outputProvider(
                camT: SIMD3<Float>(0, 1, 4),
                rotationElement: (7, 0, 1, 2.5),
                keypoint: (11, SIMD2<Float>(-20, 600))
            ),
            usedFallbackBBox: false, inputChecksum: 0,
            sourceHash: 0, bboxHash: 0, warpHash: 0
        )
        XCTAssertEqual(finiteButUnconstrained.keypoints2D[11], SIMD2<Float>(-20, 600),
                       "contract-space keypoints are not clamped to the crop")
        XCTAssertEqual(finiteButUnconstrained.globalRots[7][1][0], 2.5,
                       "finite rotations are not silently orthonormalized")
    }

    func testRetargetRejectsFiniteInputsThatOverflowDuringMarkerConstruction() {
        var outOfRange = pinnedSkeleton()
        outOfRange[0].x = 11
        XCTAssertTrue(MHRRetarget.makeBodyFrame(
            jointCoords: outOfRange, timestamp: 0, frameNumber: 0
        ).joints.isEmpty,
        "all 127 model joints must stay inside the structural metre-domain bound")

        var blendedOverflow = pinnedSkeleton()
        blendedOverflow[Idx.cSpine1].x = .greatestFiniteMagnitude
        blendedOverflow[Idx.cSpine2].x = -.greatestFiniteMagnitude
        XCTAssertTrue(MHRRetarget.makeBodyFrame(
            jointCoords: blendedOverflow, timestamp: 0, frameNumber: 0
        ).joints.isEmpty,
        "finite source joints can still overflow a blended marker and must fail before IK")

        var translatedOverflow = pinnedSkeleton()
        translatedOverflow[Idx.root].x = .greatestFiniteMagnitude
        XCTAssertTrue(MHRRetarget.makeBodyFrame(
            jointCoords: translatedOverflow,
            camT: SIMD3<Float>(.greatestFiniteMagnitude, 1, 4),
            timestamp: 0,
            frameNumber: 0
        ).joints.isEmpty,
        "finite camera composition can overflow and must fail before IK")

        XCTAssertNil(MHRRetarget.projectToImage(
            SIMD3<Float>(.greatestFiniteMagnitude, 0, 0),
            camT: SIMD3<Float>(0, 0, 4),
            imageSize: CGSize(width: 1_024, height: 1_024)
        ), "projection must not construct an infinite CGPoint")
        XCTAssertNil(MHRRetarget.projectToImage(
            .zero,
            camT: SIMD3<Float>(0, 0, 4),
            imageSize: CGSize(width: CGFloat.greatestFiniteMagnitude, height: 1_024)
        ), "non-finite Float camera intrinsics must fail closed")
    }

    // MARK: - Position variation diagnostic (not dynamics authorization)

    /// `rootTranslationObservable` reports only whether a recognized root marker
    /// varies in this input coordinate frame. A pinned stream repeats one model
    /// constant bit-for-bit. A positive result would still say nothing about
    /// gravity, camera motion, depth noise, or acceleration quality.
    func testPinnedStreamReportsRootUnobservable() {
        var detector = StaticHoldDetector()
        let names = ["PELVIS", "LHJC", "RHJC", "LKJC", "RKJC"]
        let base: [SIMD3<Double>] = [
            SIMD3(0, 0.924, 0), SIMD3(-0.085, 0.924, 0), SIMD3(0.085, 0.924, 0),
            SIMD3(-0.085, 0.504, 0), SIMD3(0.085, 0.504, 0),
        ]
        for k in 0..<SavitzkyGolayFilter.windowSize {
            // The knees move (a squat, as seen from a pinned pelvis) but PELVIS
            // never does. That is exactly the offline path's signature.
            var f = base
            f[3].y -= 0.01 * Double(k)
            f[4].y -= 0.01 * Double(k)
            detector.ingest(flatMarkerPositions: f.flatMap { [NSNumber(value: $0.x), NSNumber(value: $0.y), NSNumber(value: $0.z)] },
                            markerNames: names, timestamp: Double(k) * 0.1)
        }
        let v = detector.classify(centeredAt: 0.8)
        print("ROOT-METRIC pinned rootObservable=\(v.rootTranslationObservable) verdict=\(v.verdict)")
        XCTAssertFalse(v.rootTranslationObservable,
                       "a pinned pelvis means the root's contribution to M·q̈ is missing")

        var switchingAliases = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            let alias = k.isMultiple(of: 2) ? "PELVIS" : "MHR_ROOT"
            switchingAliases.ingest(
                flatMarkerPositions: [NSNumber(value: 0), NSNumber(value: 0.924), NSNumber(value: 0)],
                markerNames: [alias],
                timestamp: Double(k) * 0.1
            )
        }
        XCTAssertFalse(switchingAliases.classify(centeredAt: 0.8).rootTranslationObservable,
                       "switching source aliases is not evidence of physical root motion")
    }

    /// The same sequence with `cam_t` composed in flips the position diagnostic.
    /// This deliberately does NOT mean acceleration is qualified: camera motion,
    /// gravity and monocular depth remain separate typed gates.
    func testComposedStreamReportsRootObservable() {
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            // A subject walking slowly toward the camera: cam_t.z shrinks.
            let camT = SIMD3<Float>(0, 1.10, 4.34 - 0.02 * Float(k))
            let frame = MHRRetarget.makeBodyFrame(jointCoords: pinnedSkeleton(), camT: camT,
                                                  timestamp: Double(k) * 0.1, frameNumber: k)
            XCTAssertEqual(frame.dynamicsReference, .mhrCameraRelativePosition)
            var flat: [NSNumber] = []
            var names: [String] = []
            for joint in frame.joints {
                guard let markerName = JointMapping.opensimMarkerName(for: joint) else { continue }
                names.append(markerName)
                flat.append(NSNumber(value: Double(joint.worldPosition.x)))
                flat.append(NSNumber(value: Double(joint.worldPosition.y)))
                flat.append(NSNumber(value: Double(joint.worldPosition.z)))
            }
            detector.ingest(flatMarkerPositions: flat, markerNames: names,
                            timestamp: Double(k) * 0.1)
        }
        let v = detector.classify(centeredAt: 0.8)
        print("ROOT-METRIC composed rootObservable=\(v.rootTranslationObservable) "
            + "peak=\(v.peakMarkerSpeedMetersPerSecond) verdict=\(v.verdict)")
        XCTAssertTrue(v.rootTranslationObservable,
                      "with cam_t composed in, root position varies in the camera frame")
        XCTAssertEqual(v.peakMarkerSpeedMetersPerSecond, 0.2, accuracy: 1e-6,
                       "2 cm per 0.1 s of pure root translation, on every marker equally")
    }
}
