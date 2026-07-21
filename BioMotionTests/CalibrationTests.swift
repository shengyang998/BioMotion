import XCTest
@testable import BioMotion

/// Tests for the pure calibration math extracted from `CalibrationView` — the per-joint
/// quality gate, aggregate quality scoring, and height estimation/validation. These pin
/// three previously-silent failure modes:
///   1. A mostly-occluded joint (e.g. a wrist tracked in only 5% of the window) used to
///      silently contribute its noisy average as a permanent scale reference.
///   2. An implausible height estimate used to be silently replaced by a hardcoded 1.75m.
///   3. There was no signal at all when calibration quality was too low to trust.
final class CalibrationTests: XCTestCase {

    // MARK: - Helpers

    /// One joint tracked in `trackedIn` of `totalFrames` frames, all at `position` when
    /// tracked. `hips_joint` is always tracked so the frames survive the view's
    /// root-tracked prefilter (see `CalibrationView.processCalibration`).
    private func makeFrames(
        jointID: String,
        trackedIn: Int,
        totalFrames: Int,
        position: SIMD3<Float> = SIMD3(0, 1, 0)
    ) -> [BodyFrame] {
        (0..<totalFrames).map { i in
            let tracked = i < trackedIn
            let joints = [
                TrackedJoint(id: "hips_joint", name: "Pelvis", worldPosition: .zero, isTracked: true),
                TrackedJoint(id: jointID, name: jointID,
                             worldPosition: tracked ? position : .zero, isTracked: tracked),
            ]
            return BodyFrame(timestamp: Double(i), frameNumber: i, joints: joints)
        }
    }

    /// Every `JointMapping.primary` joint tracked at a fixed position in every frame,
    /// except `poorJointID` (if given), which is only tracked in the first `poorTrackedIn`
    /// frames. Lets tests isolate one joint's degradation against an otherwise-clean window.
    private func makeFullFrames(
        totalFrames: Int,
        poorJointID: String? = nil,
        poorTrackedIn: Int = 0
    ) -> [BodyFrame] {
        (0..<totalFrames).map { i in
            let joints = JointMapping.primary.map { mapping -> TrackedJoint in
                if mapping.arkitName == poorJointID {
                    let tracked = i < poorTrackedIn
                    return TrackedJoint(id: mapping.arkitName, name: mapping.displayName,
                                         worldPosition: tracked ? SIMD3(0, 1, 0) : .zero, isTracked: tracked)
                }
                return TrackedJoint(id: mapping.arkitName, name: mapping.displayName,
                                     worldPosition: SIMD3(0, 1, 0), isTracked: true)
            }
            return BodyFrame(timestamp: Double(i), frameNumber: i, joints: joints)
        }
    }

    // MARK: - (A) Per-joint quality gate

    func testJointTrackedIn3Of60FramesIsRejected() {
        let mapping = JointMapping.Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist")
        let frames = makeFrames(jointID: "left_hand_joint", trackedIn: 3, totalFrames: 60)

        let sample = CalibrationCalculator.averageJointPosition(mapping: mapping, frames: frames)

        XCTAssertEqual(sample.trackedCount, 3)
        XCTAssertFalse(sample.passesTrackingGate,
                        "3/60 (5%) tracked frames must fail the quality gate — too few, too noisy")
    }

    func testJointTrackedIn55Of60FramesIsAccepted() {
        let mapping = JointMapping.Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist")
        let frames = makeFrames(jointID: "left_hand_joint", trackedIn: 55, totalFrames: 60)

        let sample = CalibrationCalculator.averageJointPosition(mapping: mapping, frames: frames)

        XCTAssertEqual(sample.trackedCount, 55)
        XCTAssertTrue(sample.passesTrackingGate, "55/60 (~92%) tracked frames clears both the fraction and count floors")
    }

    func testJointBelowAbsoluteSampleFloorIsRejectedEvenAboveFractionThreshold() {
        // A very short capture window (e.g. app backgrounded mid-hold): 15/20 = 75% clears
        // the fraction gate but the absolute floor (20) still rejects it.
        let mapping = JointMapping.Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist")
        let frames = makeFrames(jointID: "left_hand_joint", trackedIn: 15, totalFrames: 20)

        let sample = CalibrationCalculator.averageJointPosition(mapping: mapping, frames: frames)

        XCTAssertGreaterThanOrEqual(sample.trackedFraction, CalibrationQualityConstants.minTrackedFraction)
        XCTAssertFalse(sample.passesTrackingGate, "15 samples is below the absolute floor regardless of fraction")
    }

    func testEvaluateCalibrationExcludesOnlyThePoorlyTrackedJoint() {
        let frames = makeFullFrames(totalFrames: 60, poorJointID: "left_hand_joint", poorTrackedIn: 3)

        let score = CalibrationCalculator.evaluateCalibration(frames: frames)

        XCTAssertEqual(score.rejectedJointCount, 1)
        XCTAssertEqual(score.rejectedJointNames, ["L Wrist"])
        XCTAssertEqual(score.acceptedJoints.count, JointMapping.primary.count - 1)
        // A rejected joint must not appear among accepted joints — i.e. it cannot leak
        // into the marker list that feeds nimble.scaleModel.
        XCTAssertFalse(score.acceptedJoints.contains(where: { $0.mapping.arkitName == "left_hand_joint" }))
    }

    // MARK: - (B) Quality score reflects degradation

    func testQualityScoreFallsWhenTrackedFractionsFall() {
        let cleanFrames = makeFullFrames(totalFrames: 60)
        let degradedFrames = makeFullFrames(totalFrames: 60, poorJointID: "left_hand_joint", poorTrackedIn: 30)

        let cleanScore = CalibrationCalculator.evaluateCalibration(frames: cleanFrames)
        let degradedScore = CalibrationCalculator.evaluateCalibration(frames: degradedFrames)

        XCTAssertGreaterThan(cleanScore.averageTrackedFraction, degradedScore.averageTrackedFraction)
        XCTAssertTrue(cleanScore.isAcceptable, "every joint at 100% tracked should pass")
        XCTAssertFalse(degradedScore.isAcceptable, "a joint at 50% tracked should fail and sink the overall score")
    }

    func testQualityScoreFlagsHighVarianceEvenWhenFractionPasses() {
        // Fully tracked, but the joint's position jitters across the window — i.e. the
        // subject moved or tracking was unstable during what should be a static hold.
        let mapping = JointMapping.Mapping(arkitName: "left_hand_joint", opensimName: "LWJC", displayName: "L Wrist")
        let jitteryFrames: [BodyFrame] = (0..<60).map { i in
            // Alternate between two positions 20cm apart — far more than plausible jitter
            // for a still T-pose hold.
            let position: SIMD3<Float> = i % 2 == 0 ? SIMD3(0, 1.0, 0) : SIMD3(0, 1.2, 0)
            let joints = [
                TrackedJoint(id: "hips_joint", name: "Pelvis", worldPosition: .zero, isTracked: true),
                TrackedJoint(id: "left_hand_joint", name: "L Wrist", worldPosition: position, isTracked: true),
            ]
            return BodyFrame(timestamp: Double(i), frameNumber: i, joints: joints)
        }

        let sample = CalibrationCalculator.averageJointPosition(mapping: mapping, frames: jitteryFrames)

        XCTAssertTrue(sample.passesTrackingGate, "fully tracked, so the coverage gate alone should pass")
        XCTAssertGreaterThan(sample.positionVariance, CalibrationQualityConstants.maxAcceptablePositionVariance,
                              "20cm alternation should exceed the ~2cm-std-dev variance budget")
    }

    // MARK: - (C) No silent height fallback

    func testAbsurdHeightIsRejectedNotSilentlyReplaced() {
        // Under the old behavior, an absurd estimate never surfaced at all — `estimateHeight`
        // silently substituted 1.75m and the caller had no way to distinguish "measured
        // 1.75m" from "measurement failed, defaulted to 1.75m". The validator must reject
        // the absurd values outright so the caller can tell the difference.
        XCTAssertFalse(CalibrationCalculator.isPlausibleHeight(6.0), "6m is not a human height")
        XCTAssertFalse(CalibrationCalculator.isPlausibleHeight(0.3), "0.3m is not a human height")
    }

    func testPlausibleHeightIsAccepted() {
        XCTAssertTrue(CalibrationCalculator.isPlausibleHeight(1.75))
        XCTAssertTrue(CalibrationCalculator.isPlausibleHeight(1.0))
        XCTAssertTrue(CalibrationCalculator.isPlausibleHeight(2.5))
    }

    func testEstimateFloorToCrownHeightReturnsNilWhenNoFrameIsPlausible() {
        // Head far above ankle — an impossible ~5m estimate should never surface as a
        // number the caller could mistake for a real measurement, and must not fall back
        // to a hardcoded value.
        let joints = [
            TrackedJoint(id: "head_joint", name: "Head", worldPosition: SIMD3(0, 5.0, 0), isTracked: true),
            TrackedJoint(id: "left_foot_joint", name: "L Ankle", worldPosition: SIMD3(0, 0, 0), isTracked: true),
        ]
        let frame = BodyFrame(timestamp: 0, frameNumber: 0, joints: joints)

        let estimated = CalibrationCalculator.estimateFloorToCrownHeight(from: [frame])

        XCTAssertNil(estimated, "an implausible per-frame estimate must not silently become a usable height")
    }

    func testEstimateFloorToCrownHeightReturnsPlausibleAverage() {
        let joints = [
            TrackedJoint(id: "head_joint", name: "Head", worldPosition: SIMD3(0, 1.70, 0), isTracked: true),
            TrackedJoint(id: "left_foot_joint", name: "L Ankle", worldPosition: SIMD3(0, 0.0, 0), isTracked: true),
            TrackedJoint(id: "right_foot_joint", name: "R Ankle", worldPosition: SIMD3(0, 0.0, 0), isTracked: true),
        ]
        let frame = BodyFrame(timestamp: 0, frameNumber: 0, joints: joints)

        let estimated = CalibrationCalculator.estimateFloorToCrownHeight(from: [frame])

        XCTAssertNotNil(estimated)
        // head.y (1.70) - ankle.y (0.0) + 0.08m crown offset = 1.78
        XCTAssertEqual(estimated ?? 0, 1.78, accuracy: 0.001)
        XCTAssertTrue(CalibrationCalculator.isPlausibleHeight(estimated!))
    }

    func testEstimateFloorToCrownHeightReturnsNilWithoutAnyFrames() {
        XCTAssertNil(CalibrationCalculator.estimateFloorToCrownHeight(from: []))
    }
}
