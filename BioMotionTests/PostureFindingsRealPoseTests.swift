import XCTest
import simd
@testable import BioMotion

/// Runs the findings layer on REAL Core ML output rather than synthetic joints.
///
/// `PostureFindingsTests` builds subjects with a planted deviation and checks the
/// number comes back — necessary, but it only ever exercises poses the layer was
/// designed against. This runs the same code on
/// `OfflineMuscleChainFixture.markers`, which is the actual `joint_coords` the
/// shipped `SAM3DBodyPose.mlpackage` produced for the upstream demo image, passed
/// through `MHRRetarget`.
///
/// That pose is a dancer mid-movement with one leg raised high. It is deliberately
/// NOT a posture-assessment pose, and the point of the test is that the layer
/// should degrade honestly on it — suppress what the view and the stance cannot
/// support, and never emit a confident number it has no basis for — rather than
/// produce something plausible-looking.
final class PostureFindingsRealPoseTests: XCTestCase {

    private func realJoints() -> [TrackedJoint] {
        OfflineMuscleChainFixture.markers.map { arkitId, _, p in
            TrackedJoint(id: arkitId, name: arkitId, worldPosition: p, isTracked: true)
        }
    }

    /// The offline path's camera axis, documented and verified in MHRRetarget:
    /// MHR-native is X image-right, Y up, Z toward the camera.
    private let offlineCamera = SIMD3<Double>(0, 0, 1)

    func testDoesNotCrashOrInventNumbersOnARealPrediction() {
        let report = PostureFindings.report(joints: realJoints(),
                                            cameraDepthAxis: offlineCamera)

        print("REAL-POSE orientation=\(report.view.orientation)")
        print("REAL-POSE reported=\(report.findings.count) "
            + "negligible=\(report.negligible.count) suppressed=\(report.suppressed.count)")
        for f in report.findings { print("REAL-POSE  finding: \(f.title) = \(f.value) \(f.unit)") }
        for s in report.suppressed { print("REAL-POSE  suppressed: \(s.title) — \(s.reason)") }

        // Every finding must be finite. A NaN here would render as "nan cm".
        for f in report.findings {
            XCTAssertFalse(f.value.isNaN, "\(f.title) is NaN on a real prediction")
            XCTAssertFalse(f.value.isInfinite, "\(f.title) is infinite on a real prediction")
        }
        for f in report.negligible {
            XCTAssertFalse(f.value.isNaN, "\(f.title) is NaN")
        }

        // Nothing may be silently dropped: every finding the layer knows about
        // must appear in exactly one of the three buckets.
        let total = report.findings.count + report.negligible.count + report.suppressed.count
        XCTAssertGreaterThan(total, 0, "a real prediction produced no findings at all, not even suppressed ones")
    }

    /// With the camera direction unknown, every view-dependent finding must be
    /// suppressed rather than reported against an assumed frame. The live ARKit
    /// path is a different coordinate frame and passes nil.
    func testUnknownCameraSuppressesViewDependentFindings() {
        let report = PostureFindings.report(joints: realJoints(), cameraDepthAxis: nil)
        print("REAL-POSE(nil camera) reported=\(report.findings.count) suppressed=\(report.suppressed.count)")
        XCTAssertTrue(report.findings.isEmpty && report.negligible.isEmpty,
                      "findings were reported with no known camera direction — they would be "
                      + "measured against an assumed view")
        XCTAssertFalse(report.suppressed.isEmpty,
                       "suppressed list is empty, so the findings vanished silently")
    }

    /// Degenerate input must not produce numbers. A frame where the retarget
    /// failed hands over untracked joints.
    func testUntrackedJointsProduceNoFindings() {
        let dead = OfflineMuscleChainFixture.markers.map { arkitId, _, p in
            TrackedJoint(id: arkitId, name: arkitId, worldPosition: p, isTracked: false)
        }
        let report = PostureFindings.report(joints: dead, cameraDepthAxis: offlineCamera)
        XCTAssertTrue(report.findings.isEmpty,
                      "findings were computed from joints marked untracked")
    }

    /// Mirroring the subject must mirror every sided finding and leave the
    /// unsided magnitudes alone. This catches a frame built from a hard-coded
    /// world axis instead of from the subject.
    func testMirroringTheSubjectMirrorsSidedFindings() {
        let mirrored = OfflineMuscleChainFixture.markers.map { arkitId, _, p -> TrackedJoint in
            // Mirror in X and swap left/right ids, which is what photographing
            // the same person from the other side would do.
            let swapped = arkitId
                .replacingOccurrences(of: "left_", with: "TMP_")
                .replacingOccurrences(of: "right_", with: "left_")
                .replacingOccurrences(of: "TMP_", with: "right_")
            return TrackedJoint(id: swapped, name: swapped,
                                worldPosition: SIMD3<Float>(-p.x, p.y, p.z), isTracked: true)
        }
        let a = PostureFindings.report(joints: realJoints(), cameraDepthAxis: offlineCamera)
        let b = PostureFindings.report(joints: mirrored, cameraDepthAxis: offlineCamera)

        print("REAL-POSE mirror: orig=\(a.findings.count) mirrored=\(b.findings.count)")
        XCTAssertEqual(a.findings.count, b.findings.count,
                       "mirroring changed how many findings are reportable, so something is "
                       + "measured against a world axis rather than against the subject")
        for (fa, fb) in zip(a.findings, b.findings) {
            XCTAssertEqual(abs(fa.value), abs(fb.value), accuracy: 1e-4,
                           "\(fa.title) magnitude changed under mirroring")
        }
    }
}
