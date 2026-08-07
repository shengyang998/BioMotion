import XCTest
import simd
@testable import BioMotion

/// Pins the anatomical orientation of `MuscleOverlay`'s body frame.
///
/// The third axis was computed as `right × up`, which is POSTERIOR, while the
/// comment above it said `up × right` and every entry in `muscleDefs` assumes
/// +Z is ANTERIOR. The quadriceps (recfem, vasmed, vaslat) all carry positive Z
/// offsets and the hamstrings (semimem, bflh) all carry negative ones, so the
/// quadriceps were being drawn on the back of the thigh.
///
/// A sign error in a cross product is invisible in review and produces a
/// plausible-looking render, so it is asserted here against anatomy rather than
/// against the implementation's own convention.
final class BodyFrameOrientationTests: XCTestCase {

    /// A subject standing upright and facing the camera, in ARKit world axes:
    /// X image-right, Y up, Z toward the camera.
    ///
    /// Facing the camera means their anterior is +Z, and — this is the part that
    /// makes the test discriminating — their OWN right hand is at image-LEFT,
    /// i.e. −X. A frame built from `right × up` returns −Z for this subject.
    private func facingCameraJoints() -> [TrackedJoint] {
        func j(_ id: String, _ p: SIMD3<Float>) -> TrackedJoint {
            TrackedJoint(id: id, name: id, worldPosition: p, isTracked: true)
        }
        return [
            j("hips_joint", SIMD3(0, 0.95, 0)),
            j("spine_4_joint", SIMD3(0, 1.25, 0)),
            // Subject's right hip is at image-left because they face us.
            j("right_upLeg_joint", SIMD3(-0.09, 0.92, 0)),
            j("left_upLeg_joint", SIMD3(0.09, 0.92, 0)),
        ]
    }

    func testThirdAxisIsAnteriorNotPosterior() throws {
        let frame = try XCTUnwrap(MuscleOverlay.bodyFrameForTesting(facingCameraJoints()),
                                 "body frame could not be built from an upright pose")

        // Anterior must point toward the camera (+Z) for a subject facing it.
        XCTAssertGreaterThan(frame.forward.z, 0.9,
                             "third axis points behind the subject — it is posterior, so every "
                             + "muscle def's Z offset lands on the wrong side of the body")
        XCTAssertEqual(frame.up.y, 1.0, accuracy: 1e-4)
        // And the frame's `right` must be the SUBJECT's right, at image-left.
        XCTAssertLessThan(frame.right.x, -0.9,
                          "`right` is not the subject's right — the basis is mirrored")
    }

    func testBasisIsOrthonormalAndAnatomicallyLeftHanded() throws {
        let f = try XCTUnwrap(MuscleOverlay.bodyFrameForTesting(facingCameraJoints()))
        XCTAssertEqual(simd_length(f.right), 1, accuracy: 1e-4)
        XCTAssertEqual(simd_length(f.up), 1, accuracy: 1e-4)
        XCTAssertEqual(simd_length(f.forward), 1, accuracy: 1e-4)
        XCTAssertEqual(simd_dot(f.right, f.up), 0, accuracy: 1e-4)
        XCTAssertEqual(simd_dot(f.up, f.forward), 0, accuracy: 1e-4)
        XCTAssertEqual(simd_dot(f.forward, f.right), 0, accuracy: 1e-4)

        // (right, superior, anterior) is LEFT-handed, and that is anatomy, not a
        // defect: a subject facing +Z with +Y up has their own right hand at −X,
        // so the ordered triple has determinant −1. Asserted explicitly because
        // the reflex is to "fix" it toward +1, which would mirror every offset.
        let det = simd_dot(simd_cross(f.right, f.up), f.forward)
        XCTAssertEqual(det, -1, accuracy: 1e-3,
                       "handedness flipped — the basis is mirrored and every muscle "
                       + "def's lateral offset lands on the wrong side")
    }

    /// The reason the axis matters, stated as anatomy rather than as algebra.
    func testQuadricepsRenderInFrontOfHamstrings() throws {
        let f = try XCTUnwrap(MuscleOverlay.bodyFrameForTesting(facingCameraJoints()))
        // Signs taken from `muscleDefs`: quadriceps +Z, hamstrings −Z.
        let quadOffset = SIMD3<Float>(0, 0, 0.04)     // recfem
        let hamstringOffset = SIMD3<Float>(0, 0, -0.04)  // semimem
        let quadWorld = f.transform(quadOffset)
        let hamWorld = f.transform(hamstringOffset)
        XCTAssertGreaterThan(quadWorld.z, hamWorld.z,
                             "quadriceps are being drawn behind the hamstrings")
        XCTAssertGreaterThan(quadWorld.z, 0, "quadriceps are behind the subject")
        XCTAssertLessThan(hamWorld.z, 0, "hamstrings are in front of the subject")
    }
}
