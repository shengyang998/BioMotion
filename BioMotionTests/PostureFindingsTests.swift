import CoreGraphics
import SwiftUI
import simd
import XCTest
@testable import BioMotion

/// Covers `PostureFindings` — the kinematics-only findings layer.
///
/// Three things are being pinned here, in descending order of how badly they
/// would fail silently:
///
///  1. **The anterior sign.** Every sagittal finding is a dot product with the
///     `anterior` axis. Flip that axis and "4 cm forward head" becomes "4 cm
///     behind the shoulders" while every magnitude still looks plausible — a
///     magnitude-only test would pass. `MuscleOverlay.computeBodyFrame`, which
///     this layer's basis is lifted from, computes `pelvisRight × up`, which by
///     the right-hand rule is POSTERIOR. So the sign is checked twice: once
///     analytically on a synthetic subject, and once against the real dancer
///     fixture, whose facing direction `MHRRetarget` establishes from evidence
///     external to the pose model (a photo with known facing, a mirror test,
///     and COCO keypoint naming).
///  2. **The measured value.** Synthetic subjects are built with a deviation of
///     an exactly known size, and the finding must report that number.
///  3. **View suppression.** The same deviation, viewed from an axis that
///     cannot see it, must be withheld with a reason — not reported as a
///     number that is mostly projection error.
final class PostureFindingsTests: XCTestCase {

    // MARK: - Synthetic subject

    /// A neutral standing subject, laid out in a body frame the test controls
    /// exactly, then expressed in world coordinates through a chosen
    /// orientation. Coordinates are (right, up, anterior) in metres, pelvis at
    /// the origin. Segment lengths are ordinary adult figures; nothing here
    /// depends on them being anatomically exact, only on them being consistent.
    struct Subject {
        /// The subject's own right axis, in world coordinates.
        var right: SIMD3<Double>
        /// The subject's own up (trunk) axis, in world coordinates.
        var up: SIMD3<Double>
        /// Body-frame offsets, keyed by ARKit joint id.
        var body: [String: SIMD3<Double>]

        /// `anterior = up × right` — the convention `PostureFindings` uses, and
        /// the one the dancer-fixture test independently confirms.
        var anterior: SIMD3<Double> { simd_cross(up, right) }

        static func neutral(right: SIMD3<Double> = SIMD3(1, 0, 0),
                            up: SIMD3<Double> = SIMD3(0, 1, 0)) -> Subject {
            Subject(right: right, up: up, body: [
                "hips_joint": SIMD3(0, 0, 0),
                "left_upLeg_joint": SIMD3(-0.09, 0, 0),
                "right_upLeg_joint": SIMD3(0.09, 0, 0),
                "left_leg_joint": SIMD3(-0.09, -0.42, 0),
                "right_leg_joint": SIMD3(0.09, -0.42, 0),
                "left_foot_joint": SIMD3(-0.09, -0.84, 0),
                "right_foot_joint": SIMD3(0.09, -0.84, 0),
                "left_toes_joint": SIMD3(-0.09, -0.90, 0.15),
                "right_toes_joint": SIMD3(0.09, -0.90, 0.15),
                "spine_1_joint": SIMD3(0, 0.13, 0),
                "spine_4_joint": SIMD3(0, 0.33, 0),
                "spine_7_joint": SIMD3(0, 0.52, 0),
                "neck_1_joint": SIMD3(0, 0.56, 0),
                "head_joint": SIMD3(0, 0.72, 0),
                "left_shoulder_1_joint": SIMD3(-0.18, 0.50, 0),
                "right_shoulder_1_joint": SIMD3(0.18, 0.50, 0),
                "left_forearm_joint": SIMD3(-0.20, 0.22, 0),
                "right_forearm_joint": SIMD3(0.20, 0.22, 0),
                "left_hand_joint": SIMD3(-0.21, -0.02, 0),
                "right_hand_joint": SIMD3(0.21, -0.02, 0),
            ])
        }

        /// Move one joint by (right, up, anterior) metres in the body frame.
        mutating func shift(_ id: String, _ delta: SIMD3<Double>) {
            body[id, default: .zero] += delta
        }

        func joints(omitting omitted: Set<String> = []) -> [TrackedJoint] {
            body.compactMap { id, local in
                guard !omitted.contains(id) else { return nil }
                let world = local.x * right + local.y * up + local.z * anterior
                return TrackedJoint(id: id,
                                    name: id,
                                    worldPosition: SIMD3<Float>(Float(world.x),
                                                                Float(world.y),
                                                                Float(world.z)),
                                    isTracked: true)
            }
        }
    }

    /// Camera depth axis for every test: the offline path's own constant.
    private let depth = PostureFindings.offlineCameraDepthAxis

    /// A subject facing +X. Their fore-aft axis lies in the image plane and
    /// their left-right axis lies along depth — i.e. a side-on photo.
    private func sagittalSubject() -> Subject {
        Subject.neutral(right: SIMD3(0, 0, 1), up: SIMD3(0, 1, 0))
    }

    /// A subject facing the camera (+Z). Their left-right axis lies in the
    /// image plane and their fore-aft axis lies along depth — a front-on photo.
    private func frontalSubject() -> Subject {
        Subject.neutral(right: SIMD3(-1, 0, 0), up: SIMD3(0, 1, 0))
    }

    private func finding(_ report: PostureReport, _ id: String) -> PostureFinding? {
        (report.findings + report.negligible).first { $0.id == id }
    }

    private func suppressed(_ report: PostureReport, _ id: String) -> SuppressedFinding? {
        report.suppressed.first { $0.id == id }
    }

    // MARK: - 1. The basis, and its sign

    func testTrunkBasisIsOrthonormalAndRightHanded() throws {
        let subject = sagittalSubject()
        let basis = try XCTUnwrap(PostureFindings.trunkBasis(joints: subject.joints()))

        XCTAssertEqual(simd_length(basis.up), 1, accuracy: 1e-5)
        XCTAssertEqual(simd_length(basis.right), 1, accuracy: 1e-5)
        XCTAssertEqual(simd_length(basis.anterior), 1, accuracy: 1e-5)
        XCTAssertEqual(simd_dot(basis.up, basis.right), 0, accuracy: 1e-5)
        XCTAssertEqual(simd_dot(basis.up, basis.anterior), 0, accuracy: 1e-5)
        XCTAssertEqual(simd_dot(basis.right, basis.anterior), 0, accuracy: 1e-5)

        // right × anterior = up is the right-handed ordering implied by
        // anterior = up × right. A left-handed basis here would mirror the
        // subject.
        let reconstructed = simd_cross(basis.right, basis.anterior)
        XCTAssertEqual(simd_length(reconstructed - basis.up), 0, accuracy: 1e-5)
    }

    /// The load-bearing sign check, anchored OUTSIDE the pose model.
    ///
    /// `MHRRetarget`'s file documentation establishes, from a photo with
    /// externally-known facing plus a mirror test plus COCO keypoint naming,
    /// that the dancer in `OfflineMuscleChainFixture` faces IMAGE-RIGHT, and
    /// that MHR-native +X is image-right. So the computed anterior axis must
    /// have a positive X component. It also records her right hip and shoulder
    /// as nearer the camera than her left (+Z is toward the camera), which
    /// gives the `right` axis an independent second anchor.
    func testAnteriorSignAgreesWithTheDancersKnownFacing() throws {
        let joints = OfflineMuscleChainFixture.markers.map {
            TrackedJoint(id: $0.0, name: $0.1, worldPosition: $0.2, isTracked: true)
        }
        let basis = try XCTUnwrap(PostureFindings.trunkBasis(joints: joints))

        print("BASIS-METRIC dancer anterior=(\(basis.anterior.x), \(basis.anterior.y), \(basis.anterior.z))")
        print("BASIS-METRIC dancer right=(\(basis.right.x), \(basis.right.y), \(basis.right.z))")

        XCTAssertGreaterThan(basis.anterior.x, 0,
                             "anterior points image-LEFT, but MHRRetarget establishes from a photo with known facing that this subject faces image-right — the anterior axis is sign-flipped")
        XCTAssertGreaterThan(basis.right.z, 0,
                             "the subject's right side is placed away from the camera, but MHRRetarget measured her right hip/shoulder NEARER the camera (Z_r +0.065/+0.123 vs Z_l -0.058/-0.083)")
    }

    /// Same sign, checked analytically rather than by fixture: a subject built
    /// facing +X must produce an anterior axis along +X.
    func testAnteriorSignOnAConstructedSubject() throws {
        let subject = sagittalSubject()      // built facing +X
        let basis = try XCTUnwrap(PostureFindings.trunkBasis(joints: subject.joints()))
        XCTAssertEqual(basis.anterior.x, 1, accuracy: 1e-5)
    }

    // MARK: - 2. Known deviations are reported at their known size

    func testForwardHeadReportsFiveCentimetres() throws {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))   // +5 cm anterior

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "forward_head"),
                              "forward head was not reported; suppressed: \(report.suppressed.map(\.reason))")

        XCTAssertEqual(f.value, 5.0, accuracy: 1e-3)
        XCTAssertEqual(f.unit, .centimetres)
        XCTAssertEqual(f.sideMeaning, "ahead of the shoulders")
        XCTAssertEqual(f.formattedValue, "5.0 cm")
        // Honesty requirement: the row must name what it is measured between.
        XCTAssertTrue(f.measuredBetween.contains("shoulder"), f.measuredBetween)
        // A side-on view puts the fore-aft axis fully in the image plane.
        XCTAssertEqual(f.depthFraction, 0, accuracy: 1e-5)
    }

    func testBackwardHeadFlipsTheDirectionWord() throws {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, -0.04))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "forward_head"))
        XCTAssertEqual(f.value, -4.0, accuracy: 1e-3)
        XCTAssertEqual(f.sideMeaning, "behind the shoulders")
        XCTAssertEqual(f.formattedValue, "4.0 cm", "the displayed number is a magnitude; the direction is a word")
    }

    func testRoundedShouldersMeasuresProtractionAgainstC7() throws {
        var subject = sagittalSubject()
        subject.shift("left_shoulder_1_joint", SIMD3(0, 0, 0.03))
        subject.shift("right_shoulder_1_joint", SIMD3(0, 0, 0.03))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "rounded_shoulders"))
        XCTAssertEqual(f.value, 3.0, accuracy: 1e-3)
        XCTAssertEqual(f.sideMeaning, "shoulders ahead of C7")
    }

    func testShoulderHeightAsymmetryCarriesTheCorrectSide() throws {
        var subject = frontalSubject()
        subject.shift("left_shoulder_1_joint", SIMD3(0, 0.03, 0))   // left 3 cm higher

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "shoulder_height"),
                              "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")
        XCTAssertEqual(f.value, 3.0, accuracy: 1e-3)
        XCTAssertEqual(f.side, PostureSide.left)
        XCTAssertEqual(f.sideMeaning, "left shoulder higher")

        // And the mirror image must report the other side, not just the other sign.
        var mirrored = frontalSubject()
        mirrored.shift("right_shoulder_1_joint", SIMD3(0, 0.03, 0))
        let mirroredReport = PostureFindings.report(joints: mirrored.joints(), cameraDepthAxis: depth)
        let m = try XCTUnwrap(finding(mirroredReport, "shoulder_height"))
        XCTAssertEqual(m.side, PostureSide.right)
        XCTAssertEqual(m.magnitude, 3.0, accuracy: 1e-3)
    }

    func testLateralHeadTiltReportsAKnownAngleAndSide() throws {
        var subject = frontalSubject()
        // neck→head is 0.16 m of pure up. Tip the head 0.16·tan(10°) toward the
        // subject's right and the segment sits at exactly 10°.
        let dx = 0.16 * tan(10.0 * .pi / 180)
        subject.shift("head_joint", SIMD3(dx, 0, 0))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "head_tilt"),
                              "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")
        XCTAssertEqual(f.value, 10.0, accuracy: 0.05)
        XCTAssertEqual(f.side, PostureSide.right)
        XCTAssertEqual(f.unit, .degrees)
    }

    func testLateralWeightShiftMeasuresPelvisOverAnkleMidpoint() throws {
        var subject = frontalSubject()
        // Move BOTH ankles 4 cm to the subject's left => the pelvis sits 4 cm
        // to the subject's right of the ankle midpoint.
        subject.shift("left_foot_joint", SIMD3(-0.04, 0, 0))
        subject.shift("right_foot_joint", SIMD3(-0.04, 0, 0))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "weight_shift"),
                              "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")
        XCTAssertEqual(f.value, 4.0, accuracy: 1e-3)
        XCTAssertEqual(f.side, PostureSide.right)
    }

    func testTrunkLeanIsMeasuredAgainstTheLegAxisNotTheTrunkAxis() throws {
        // Tip the WHOLE trunk (spine chain, shoulders, neck, head) forward
        // about the pelvis. Referenced to the trunk's own axis this is
        // invisible by construction; referenced to the leg axis it is a lean.
        var subject = sagittalSubject()
        let tilt = 8.0 * .pi / 180
        for id in ["spine_1_joint", "spine_4_joint", "spine_7_joint", "neck_1_joint",
                   "head_joint", "left_shoulder_1_joint", "right_shoulder_1_joint"] {
            let p = subject.body[id]!
            // rotate (up, anterior) about the right axis
            subject.body[id] = SIMD3(p.x,
                                     p.y * cos(tilt) - p.z * sin(tilt),
                                     p.y * sin(tilt) + p.z * cos(tilt))
        }

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)

        // Shoulder midpoint is 0.50 m up the trunk; an 8° tilt puts it
        // 0.50·sin(8°) = 6.96 cm ahead of the pelvis.
        let lean = try XCTUnwrap(finding(report, "trunk_lean_sagittal"),
                                 "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")
        XCTAssertEqual(lean.value, 50 * sin(tilt), accuracy: 0.05)
        XCTAssertEqual(lean.sideMeaning, "shoulders ahead of the pelvis")

        // ...while protraction against the trunk axis stays ~0, because the
        // trunk axis rotated with the trunk. This is what makes the two
        // findings independent rather than duplicates.
        let protraction = try XCTUnwrap(finding(report, "rounded_shoulders"))
        XCTAssertEqual(protraction.value, 0, accuracy: 0.2)
        XCTAssertTrue(report.negligible.contains { $0.id == "rounded_shoulders" },
                      "a ~0 protraction must be grouped as 'no measurable deviation', not headlined")
    }

    func testKyphosisProxyReportsChainDeflectionAndSaysItIsNotPerVertebra() throws {
        var subject = sagittalSubject()
        // Tip the mid-spine→C7 segment forward by moving C7 and everything
        // above it. C7 sits 0.19 m above the mid-spine marker.
        let bend = 12.0 * .pi / 180
        let sM = subject.body["spine_4_joint"]!
        for id in ["spine_7_joint", "neck_1_joint", "head_joint",
                   "left_shoulder_1_joint", "right_shoulder_1_joint"] {
            let rel = subject.body[id]! - sM
            subject.body[id] = sM + SIMD3(rel.x,
                                          rel.y * cos(bend) - rel.z * sin(bend),
                                          rel.y * sin(bend) + rel.z * cos(bend))
        }

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "kyphosis_proxy"),
                              "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")

        // The trunk axis itself moved (C7 moved), so the reported chain angle
        // is the angle BETWEEN the two segments and is reference-free.
        XCTAssertEqual(f.value, 12.0, accuracy: 0.5)
        XCTAssertEqual(f.sideMeaning, "upper back tipped further forward than the lower back")

        let caveat = try XCTUnwrap(f.caveat)
        XCTAssertTrue(caveat.contains("not a measurement of individual vertebrae"),
                      "the upper-back estimate must state its user-facing limitation")
        XCTAssertFalse(caveat.contains("STATUS.md"))
        XCTAssertFalse(caveat.contains("null model"))
    }

    func testTransverseRotationReportsAKnownTwistWithItsNoiseGain() throws {
        var subject = sagittalSubject()
        // Rotate the shoulder line 15° about the trunk axis.
        let twist = 15.0 * .pi / 180
        for id in ["left_shoulder_1_joint", "right_shoulder_1_joint"] {
            let p = subject.body[id]!
            subject.body[id] = SIMD3(p.x * cos(twist) - p.z * sin(twist),
                                     p.y,
                                     p.x * sin(twist) + p.z * cos(twist))
        }

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        let f = try XCTUnwrap(finding(report, "transverse_rotation"),
                              "suppressed: \(report.suppressed.map { "\($0.id): \($0.reason)" })")
        XCTAssertEqual(f.magnitude, 15.0, accuracy: 0.2)

        // The baseline/noise-gain caveat is the honest form of "this number is
        // a small difference over a short lever".
        let caveat = try XCTUnwrap(f.caveat)
        XCTAssertTrue(caveat.contains("shoulder baseline"), caveat)
        XCTAssertTrue(caveat.contains("°"), caveat)
    }

    // MARK: - 3. The null case

    func testPerfectPostureReportsNothing() {
        for subject in [sagittalSubject(), frontalSubject()] {
            let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
            XCTAssertTrue(report.findings.isEmpty,
                          "a symmetric neutral subject produced findings: \(report.findings.map { "\($0.id)=\($0.value)" })")
            XCTAssertFalse(report.negligible.isEmpty,
                           "the measurements must still be computed and listed as negligible, not silently dropped")
            for f in report.negligible {
                XCTAssertLessThan(f.magnitude, 1.0, "\(f.id) = \(f.value)")
            }
        }
    }

    // MARK: - 4. View suppression — the biggest correctness risk

    func testForwardHeadIsSuppressedWhenTheSubjectFacesTheCamera() throws {
        var subject = frontalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))   // the SAME 5 cm deviation

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)

        XCTAssertNil(finding(report, "forward_head"),
                     "a 5 cm fore-aft offset measured along the camera's depth axis must not be reported as a number")
        let s = try XCTUnwrap(suppressed(report, "forward_head"))
        XCTAssertTrue(s.reason.contains("side-on"), s.reason)
        XCTAssertTrue(s.reason.contains("depth"), s.reason)
        XCTAssertEqual(report.view.orientation, .frontal)
    }

    func testShoulderAsymmetryIsSuppressedFromASideOnView() throws {
        var subject = sagittalSubject()
        subject.shift("left_shoulder_1_joint", SIMD3(0, 0.03, 0))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        XCTAssertNil(finding(report, "shoulder_height"))
        let s = try XCTUnwrap(suppressed(report, "shoulder_height"))
        XCTAssertTrue(s.reason.contains("front-on"), s.reason)
        XCTAssertEqual(report.view.orientation, .sagittal)
    }

    /// The corrected transverse-rotation geometry: its sensitive direction is
    /// the NORMAL to the shoulder line, so it needs a side view, not a front
    /// view. An earlier version of the implementation hardcoded this finding's
    /// depth fraction to 1.0 on the (wrong) reasoning that rotation about the
    /// vertical is always a depth measurement.
    func testTransverseRotationNeedsASideViewNotAFrontView() throws {
        func twisted(_ base: Subject) -> Subject {
            var s = base
            let twist = 15.0 * .pi / 180
            for id in ["left_shoulder_1_joint", "right_shoulder_1_joint"] {
                let p = s.body[id]!
                s.body[id] = SIMD3(p.x * cos(twist) - p.z * sin(twist), p.y,
                                   p.x * sin(twist) + p.z * cos(twist))
            }
            return s
        }

        let sideOn = PostureFindings.report(joints: twisted(sagittalSubject()).joints(),
                                            cameraDepthAxis: depth)
        let supported = try XCTUnwrap(finding(sideOn, "transverse_rotation"))
        XCTAssertLessThan(supported.depthFraction, 0.5)

        let frontOn = PostureFindings.report(joints: twisted(frontalSubject()).joints(),
                                             cameraDepthAxis: depth)
        XCTAssertNil(finding(frontOn, "transverse_rotation"))
        XCTAssertNotNil(suppressed(frontOn, "transverse_rotation"))
    }

    func testObliqueViewWithholdsBothAxes() throws {
        // 45° of yaw: neither the fore-aft nor the left-right axis lies in the
        // image plane, so neither family of findings can be trusted.
        let s = 1.0 / 2.0.squareRoot()
        var subject = Subject.neutral(right: SIMD3(-s, 0, s), up: SIMD3(0, 1, 0))
        subject.shift("head_joint", SIMD3(0, 0, 0.05))
        subject.shift("left_shoulder_1_joint", SIMD3(0, 0.03, 0))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        XCTAssertEqual(report.view.orientation, .oblique)
        XCTAssertNil(finding(report, "forward_head"))
        XCTAssertNil(finding(report, "shoulder_height"))
        XCTAssertTrue(report.findings.isEmpty,
                      "nothing should survive a 45° view: \(report.findings.map(\.id))")
    }

    func testUnknownCameraDirectionSuppressesEverything() {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: nil)
        XCTAssertTrue(report.findings.isEmpty)
        XCTAssertTrue(report.negligible.isEmpty)
        XCTAssertFalse(report.suppressed.isEmpty)
        XCTAssertEqual(report.view.orientation, .undetermined)
        for s in report.suppressed {
            XCTAssertTrue(s.reason.contains("camera direction"), "\(s.id): \(s.reason)")
        }
    }

    /// Camera roll rotates the whole scene about the depth axis. Nothing about
    /// the subject or the view changed, so no finding may change — this is what
    /// resolving everything in a body frame buys, and it is the property that
    /// would break if any measurement quietly used world +Y as "up".
    func testFindingsAreInvariantToCameraRoll() throws {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))
        let upright = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)

        let roll = 20.0 * .pi / 180
        let rolled = subject.joints().map { j -> TrackedJoint in
            let p = j.worldPosition
            return TrackedJoint(id: j.id, name: j.name,
                                worldPosition: SIMD3<Float>(
                                    Float(Double(p.x) * cos(roll) - Double(p.y) * sin(roll)),
                                    Float(Double(p.x) * sin(roll) + Double(p.y) * cos(roll)),
                                    p.z),
                                isTracked: true,
                                opensimMarkerNameOverride: j.opensimMarkerNameOverride)
        }
        let rolledReport = PostureFindings.report(joints: rolled, cameraDepthAxis: depth)

        XCTAssertEqual(rolledReport.view.orientation, upright.view.orientation)
        XCTAssertEqual(rolledReport.findings.map(\.id), upright.findings.map(\.id))
        for (a, b) in zip(upright.findings, rolledReport.findings) {
            XCTAssertEqual(a.value, b.value, accuracy: 1e-2, a.id)
            XCTAssertEqual(a.depthFraction, b.depthFraction, accuracy: 1e-3, a.id)
        }
    }

    // MARK: - 5. Ranking and presentation contract

    func testFindingsAreRankedByMagnitude() {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.06))            // 6 cm
        subject.shift("left_shoulder_1_joint", SIMD3(0, 0, 0.02)) // contributes 1 cm
        subject.shift("right_shoulder_1_joint", SIMD3(0, 0, 0.02))

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        XCTAssertGreaterThan(report.findings.count, 1)
        for (a, b) in zip(report.findings, report.findings.dropFirst()) {
            XCTAssertGreaterThanOrEqual(a.magnitude, b.magnitude,
                                        "\(a.id) (\(a.magnitude)) must not rank below \(b.id) (\(b.magnitude))")
        }
        XCTAssertEqual(report.findings.first?.id, "forward_head")
    }

    /// The honesty contract, pinned so a later UI edit cannot quietly drop it.
    func testNoFindingCarriesAVerdictAndTheNoRangeNoteIsAlwaysVisible() {
        XCTAssertTrue(PostureFindings.alwaysVisibleNote.contains("No normal range"),
                      PostureFindings.alwaysVisibleNote)
        XCTAssertTrue(PostureFindings.alwaysVisibleNote.lowercased().contains("not diagnoses"),
                      PostureFindings.alwaysVisibleNote)
        XCTAssertFalse(PostureFindings.methodNotes.isEmpty)

        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))
        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)

        for f in report.findings {
            XCTAssertFalse(f.measuredBetween.isEmpty, "\(f.id) has no stated landmarks")
            XCTAssertFalse(f.headline.isEmpty)
            // Every surfaced number carries its projection exposure.
            XCTAssertTrue(f.projectionNote.contains("depth"), f.projectionNote)
            // No verdict vocabulary anywhere in a row's text.
            let text = "\(f.title) \(f.sideMeaning) \(f.measuredBetween) \(f.caveat ?? "")".lowercased()
            for word in ["normal", "abnormal", "healthy", "poor posture", "should be", "ideal"] {
                XCTAssertFalse(text.contains(word), "\(f.id) uses verdict wording '\(word)': \(text)")
            }
        }
    }

    // MARK: - 6. Degenerate input

    func testMissingJointsAreReportedAsSuppressedNotCrashed() throws {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))
        let report = PostureFindings.report(
            joints: subject.joints(omitting: ["head_joint", "left_foot_joint"]),
            cameraDepthAxis: depth)

        let head = try XCTUnwrap(suppressed(report, "forward_head"))
        XCTAssertTrue(head.reason.contains("missing"), head.reason)
        let shift = try XCTUnwrap(suppressed(report, "weight_shift"))
        XCTAssertTrue(shift.reason.contains("ankle"), shift.reason)
    }

    func testNoTrunkReferenceFrameReportsOneClearReason() {
        let joints = [
            TrackedJoint(id: "hips_joint", name: "p", worldPosition: SIMD3(0, 0, 0), isTracked: true),
            TrackedJoint(id: "left_upLeg_joint", name: "l", worldPosition: SIMD3(-0.09, 0, 0), isTracked: true),
        ]
        let report = PostureFindings.report(joints: joints, cameraDepthAxis: depth)
        XCTAssertTrue(report.findings.isEmpty)
        XCTAssertEqual(report.suppressed.count, 1)
        XCTAssertEqual(report.suppressed.first?.id, "basis")
        XCTAssertEqual(report.view.orientation, .undetermined)
    }

    /// Non-standing poses must not be handed a leg-axis plumb line. The dancer
    /// fixture is the real case: a raised leg makes the ankle midpoint useless
    /// as a base of support.
    func testSeatedSubjectHasNoPlumbLine() throws {
        var subject = sagittalSubject()
        // Fold the legs forward so the leg axis is ~horizontal.
        subject.body["left_foot_joint"] = SIMD3(-0.09, 0.02, 0.60)
        subject.body["right_foot_joint"] = SIMD3(0.09, 0.02, 0.60)

        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        for id in ["trunk_lean_sagittal", "trunk_lean_lateral", "weight_shift"] {
            let s = try XCTUnwrap(suppressed(report, id), "\(id) was not suppressed")
            XCTAssertTrue(s.reason.contains("standing over their feet"), s.reason)
        }
    }

    // MARK: - 7. The panel actually draws

    /// Renders `PostureFindingsPanel` off-screen and checks that ink hit the
    /// canvas. This is not a design review — it is the cheapest available proof
    /// that the view lays out and draws rather than crashing or coming back
    /// blank, which unit tests on the model alone cannot tell you.
    @MainActor
    func testPanelRendersSomething() throws {
        var subject = sagittalSubject()
        subject.shift("head_joint", SIMD3(0, 0, 0.05))
        let report = PostureFindings.report(joints: subject.joints(), cameraDepthAxis: depth)
        XCTAssertFalse(report.findings.isEmpty)

        let renderer = ImageRenderer(content: PostureFindingsPanel(report: report)
            .frame(width: 390, height: 300))
        renderer.scale = 1
        let image = try XCTUnwrap(renderer.uiImage, "the panel produced no image at all")
        XCTAssertEqual(image.size.width, 390, accuracy: 1)
        XCTAssertEqual(image.size.height, 300, accuracy: 1)

        // A blank render would be a single flat colour end to end.
        let cg = try XCTUnwrap(image.cgImage)
        var pixels = [UInt8](repeating: 0, count: cg.width * cg.height * 4)
        let ctx = try XCTUnwrap(CGContext(data: &pixels,
                                          width: cg.width, height: cg.height,
                                          bitsPerComponent: 8, bytesPerRow: cg.width * 4,
                                          space: CGColorSpaceCreateDeviceRGB(),
                                          bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue))
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: cg.width, height: cg.height))
        var distinct = Set<UInt32>()
        var i = 0
        while i + 2 < pixels.count {
            let r = UInt32(pixels[i]) << 16
            let g = UInt32(pixels[i + 1]) << 8
            let b = UInt32(pixels[i + 2])
            distinct.insert(r | g | b)
            i += 4
        }
        print("RENDER-METRIC panel distinct_colours=\(distinct.count)")
        XCTAssertGreaterThan(distinct.count, 8, "the panel rendered as a flat colour — nothing was drawn")
    }

    // MARK: - 8. Real pose-model output

    /// End-to-end on the real Core ML dancer markers. She is in a strongly
    /// oblique, twisted pose, so this documents the MEASURED behaviour of the
    /// view gate on real data rather than asserting a posture number.
    func testDancerFixtureIsClassifiedAndDoesNotFabricateNumbers() {
        let joints = OfflineMuscleChainFixture.markers.map {
            TrackedJoint(id: $0.0, name: $0.1, worldPosition: $0.2, isTracked: true)
        }
        let report = PostureFindings.report(joints: joints, cameraDepthAxis: depth)

        print("VIEW-METRIC dancer orientation=\(report.view.orientation) " +
              "anteriorDepth=\(report.view.anteriorDepthFraction ?? -1) " +
              "lateralDepth=\(report.view.lateralDepthFraction ?? -1) " +
              "verticalDepth=\(report.view.verticalDepthFraction ?? -1)")
        print("VIEW-METRIC dancer reported=\(report.findings.map(\.id)) " +
              "negligible=\(report.negligible.map(\.id)) " +
              "suppressed=\(report.suppressed.map(\.id))")

        XCTAssertEqual(report.view.orientation, .oblique,
                       "this pose is turned ~45° to the camera; if it ever classifies as sagittal or frontal the gate has moved")
        // Whatever survives must be consistent with the gate that produced it.
        for f in report.findings + report.negligible {
            XCTAssertLessThanOrEqual(f.depthFraction, PostureFindings.depthSuppressionFraction,
                                     "\(f.id) was surfaced with depthFraction \(f.depthFraction)")
        }
        XCTAssertFalse(report.suppressed.isEmpty)
    }
}
