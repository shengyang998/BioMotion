import simd
import XCTest
@testable import BioMotion

/// Covers the root translation the offline pose source drops, and the engine's
/// ability to tell whether the stream it is being fed carries it.
///
/// # The finding these tests encode
/// `joint_coords` zeroes `global_trans`, so the pelvis is pinned at the model
/// constant (0, 0.924, 0) in every frame. That is NOT a limitation of the pose
/// model: the model emits the translation separately as `cam_t`, the app
/// already exports it, already stores it on `FrameResult`, and already uses it
/// to project the overlay. `MHRRetarget.makeBodyFrame(jointCoords:camT:…)`
/// composes it back in.
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

    // MARK: - The default is byte-for-byte the old behaviour

    /// The parameter is opt-in. Every existing caller omits it and must get the
    /// pelvis-pinned frame it got before the parameter existed — otherwise this
    /// change would silently alter the live ARKit path's sibling and every
    /// fixture in the suite.
    func testOmittingCamTIsIdenticalToPassingNil() {
        let j = pinnedSkeleton()
        let a = MHRRetarget.makeBodyFrame(jointCoords: j, timestamp: 1.0, frameNumber: 3)
        let b = MHRRetarget.makeBodyFrame(jointCoords: j, camT: nil, timestamp: 1.0, frameNumber: 3)
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

    /// The composition is a pure translation: it moves where the body IS
    /// without touching what the body is DOING. Every inter-marker distance —
    /// i.e. everything IK fits and every joint angle downstream — is unchanged
    /// to float precision. That is what makes this safe to switch on: it can
    /// only add the missing 3 root translational DOFs.
    func testCamTIsARigidTranslationAndChangesNoRelativeGeometry() {
        let j = pinnedSkeleton()
        let camT = SIMD3<Float>(0.31, 1.10, 4.34)      // the measured scale on real video
        let pinned = positions(MHRRetarget.makeBodyFrame(jointCoords: j, timestamp: 0, frameNumber: 0))
        let world = positions(MHRRetarget.makeBodyFrame(jointCoords: j, camT: camT,
                                                        timestamp: 0, frameNumber: 0))
        let expected = MHRRetarget.rootTranslation(camT: camT)

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

        var maxPixelGap: CGFloat = 0
        var compared = 0
        for id in pinned.keys.sorted() {
            guard let old = MHRRetarget.projectToImage(pinned[id]!, camT: camT, imageSize: size),
                  let new = MHRRetarget.projectToImage(world[id]!, camT: .zero, imageSize: size)
            else { continue }
            maxPixelGap = max(maxPixelGap, hypot(old.x - new.x, old.y - new.y))
            compared += 1
        }
        print("ROOT-METRIC projection compared=\(compared) max_pixel_gap=\(maxPixelGap)")
        XCTAssertEqual(compared, 20, "all twenty markers must project in front of the camera")
        XCTAssertLessThan(maxPixelGap, 0.5,
                          "composing cam_t must be the same transform the validated projection applies")
    }

    /// The depth sign is the one that is easy to get backwards and expensive to
    /// get wrong (it would put the subject behind the camera). `cam_t.z` is a
    /// POSITIVE distance in front of the camera in the OpenCV-style frame, so
    /// in this file's Y-up frame — camera at the origin looking along −Z — the
    /// subject must land at NEGATIVE z, and BELOW the optical axis for a phone
    /// held above hip height.
    func testRootTranslationSigns() {
        let t = MHRRetarget.rootTranslation(camT: SIMD3<Float>(0.31, 1.10, 4.34))
        print("ROOT-METRIC signs \(t)")
        XCTAssertEqual(t.x, 0.31, accuracy: 1e-6, "lateral passes through unchanged")
        XCTAssertEqual(t.y, -1.10, accuracy: 1e-6, "cam_t.y is DOWN-positive; the Y-up frame negates it")
        XCTAssertEqual(t.z, -4.34, accuracy: 1e-6, "cam_t.z is AWAY-positive; −Z is in front of the camera")
    }

    // MARK: - The engine can tell whether it was given the translation

    /// `rootTranslationObservable` reads the data, not a flag. A pinned stream
    /// repeats one model constant bit-for-bit, so there is no way for a flag and
    /// the stream to disagree — which matters because the flag would have to be
    /// set in `OfflineSessionRunner`, a file this change does not own.
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

    /// The same sequence with `cam_t` composed in flips it. Built through
    /// `makeBodyFrame` rather than by hand so the test exercises the real
    /// composition, not a restatement of it.
    func testComposedStreamReportsRootObservable() {
        var detector = StaticHoldDetector()
        for k in 0..<SavitzkyGolayFilter.windowSize {
            // A subject walking slowly toward the camera: cam_t.z shrinks.
            let camT = SIMD3<Float>(0, 1.10, 4.34 - 0.02 * Float(k))
            let frame = MHRRetarget.makeBodyFrame(jointCoords: pinnedSkeleton(), camT: camT,
                                                  timestamp: Double(k) * 0.1, frameNumber: k)
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
                      "with cam_t composed in, the root moves and its acceleration is observable")
        XCTAssertEqual(v.peakMarkerSpeedMetersPerSecond, 0.2, accuracy: 1e-6,
                       "2 cm per 0.1 s of pure root translation, on every marker equally")
    }
}
