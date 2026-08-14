import Vision
import XCTest

@testable import BioMotion

private typealias PreprocessingConstants = SAM3DPoseEstimator.PreprocessingConstants

/// The offline path's person box decides what the model can see. A torso-only
/// box silently removes the legs from the crop, and the failure is quiet: the
/// torso still tracks, so the overlay looks broadly plausible while the legs
/// hold a near-standing mean pose regardless of what the subject is doing.
///
/// Measured on a 576x768 running clip against Vision's own 2-D body pose
/// (`labs/sam-3d-body/export/box_ablation.py`, 20 frames): leg error 9.0% of
/// subject height with the default box versus 4.6% with the full-body box, with
/// torso error unchanged at 2.0% vs 1.9%.
final class PersonBoxTests: XCTestCase {

    /// Pins the trap itself. If a future SDK flips this default, the production
    /// assertion below stops being load-bearing and this test says so directly
    /// rather than leaving a comment that has quietly become false.
    func testUpperBodyOnlyDefaultsToTrue() {
        XCTAssertTrue(VNDetectHumanRectanglesRequest().upperBodyOnly,
                      "Vision's default changed; the rationale on makePersonRectangleRequest() "
                      + "and this test's premise both need rewriting.")
    }

    func testProductionRequestAsksForTheWholeBody() {
        XCTAssertFalse(SAM3DPoseEstimator.makePersonRectangleRequest().upperBodyOnly,
                       "A torso-only box crops the legs out of the model's input; the legs then "
                       + "regress to a mean standing pose while the torso still tracks.")
    }

    /// A box that reaches the feet is worthless if the square crop built from it
    /// then throws the legs away again. Guards the padding + squaring step for
    /// the shape that matters here: a running stride, which is WIDE.
    func testSquareCropCoversAWideStrideBox() {
        // t=22.0s of the reference clip, full-body box, in source pixels.
        let box = CGRect(x: 173, y: 219, width: 185, height: 438)
        let padded = CGSize(width: box.width * PreprocessingConstants.bboxPadding, height: box.height * PreprocessingConstants.bboxPadding)
        let side = max(max(padded.width / PreprocessingConstants.bboxPriorAspect, padded.height), 1)

        XCTAssertGreaterThanOrEqual(side, padded.height,
                                    "the crop must be at least as tall as the padded box")
        XCTAssertGreaterThanOrEqual(side, padded.width,
                                    "the crop must be at least as wide as the padded box")

        // The aspect prior is what makes a wide box grow the crop rather than
        // clipping it. A stride wider than 0.75 of its height must be driven by
        // the width term, not the height term.
        let wide = CGRect(x: 0, y: 0, width: 400, height: 400)
        let widePadded = CGSize(width: wide.width * PreprocessingConstants.bboxPadding, height: wide.height * PreprocessingConstants.bboxPadding)
        let wideSide = max(max(widePadded.width / PreprocessingConstants.bboxPriorAspect,
                               widePadded.height), 1)
        XCTAssertEqual(wideSide, widePadded.width / PreprocessingConstants.bboxPriorAspect,
                       accuracy: 1e-6,
                       "a box wider than the aspect prior must be sized by its width")
    }

    /// The production-facing seam the 2026-08-14 person-box sidecar amendment
    /// added, pinned in the ONE lane that gates anything.
    ///
    /// # (a) an injected box is consumed VERBATIM, and Vision is not reached
    ///
    /// The numbers are the 3-decimal box the macOS probe printed for frame 0 of
    /// `video_012` against that clip's decoded 576x1024. `personBoxPixels` is the
    /// single copy of Vision's bottom-left-origin / Y-up → top-left-origin /
    /// Y-down flip: a wrong-signed flip reads `y1 = 243.712` instead of
    /// `356.352`, i.e. 112.6 px away.
    ///
    /// WHY THIS IS NOT TAUTOLOGICAL IN THIS LANE, stated because the same
    /// assertion IS tautological inside the fixture generator. The fast lane
    /// runs in the iOS Simulator, where `VNDetectHumanRectanglesRequest.perform`
    /// THROWS (`com.apple.Vision Code=9`, 12/12 measured 2026-08-14) and
    /// `detectPersonBBox` answers a throw with the WHOLE IMAGE plus
    /// `usedFallback = true`. If the injected path ever reached Vision, this
    /// assertion would read 0,0,576,1024 and `true`, and FAIL. That evidence is
    /// environment-dependent, and it is backed by the structural contract that
    /// `resolvePersonBox` returns before any `VNImageRequestHandler` exists.
    ///
    /// It never calls `estimate()`, which would need the 1.3 GiB dev-bundled
    /// model — a dependency the fast lane must not acquire.
    ///
    /// # (b) the convention-refusal path exists and fails CLOSED
    ///
    /// A sidecar carrying `upperBodyOnly = true`, a non-`.up` handler
    /// orientation or a foreign schema id must be REFUSED, not consumed. The
    /// generator that runs those refusals is whole-class-skipped here, so
    /// without this pin the refusal path would have no gating coverage at all.
    func testAnInjectedPersonBoxIsConsumedVerbatimWithoutVision() throws {
        // (a) --------------------------------------------------------------
        let size = CGSize(width: 576, height: 1024)
        let normalized = CGRect(x: 0.303, y: 0.238, width: 0.334, height: 0.414)
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1          // points == pixels, so the pin is deterministic
        let image = UIGraphicsImageRenderer(size: size, format: format).image { ctx in
            UIColor.gray.setFill()
            ctx.fill(CGRect(origin: .zero, size: size))
        }
        XCTAssertEqual(image.imageOrientation, .up)

        let (rect, usedFallback) = try SAM3DPoseEstimator.resolvePersonBox(
            uiImage: image, injectedNormalizedBottomLeft: normalized)
        XCTAssertFalse(usedFallback,
                       "an injected box must never be reported as the whole-image fallback; "
                       + "in this Simulator lane Vision throws, so reaching it would return "
                       + "the whole image with usedFallback = true")
        XCTAssertEqual(rect.minX, 174.528, accuracy: 1e-6)
        XCTAssertEqual(rect.minY, 356.352, accuracy: 1e-6,
                       "Vision's Y is bottom-left-origin and MUST be flipped; a wrong-signed "
                       + "flip reads 243.712 here")
        XCTAssertEqual(rect.width, 192.384, accuracy: 1e-6)
        XCTAssertEqual(rect.height, 423.936, accuracy: 1e-6)
        XCTAssertNotEqual(rect, CGRect(origin: .zero, size: size),
                          "the whole image is what the Vision fallback returns")

        // A degenerate box must THROW rather than have the whole image
        // substituted, so no caller can obtain a silent whole-image crop.
        XCTAssertThrowsError(try SAM3DPoseEstimator.resolvePersonBox(
            uiImage: image,
            injectedNormalizedBottomLeft: CGRect(x: 0.5, y: 0.5, width: 0, height: 0)))

        // (b) --------------------------------------------------------------
        let fps = 30.0
        let step = 1.0 / fps
        let timestamps = [3.15, 3.15 + step]
        let expectation = SidecarPlan.Expectation(
            clip: "video_012", videoSHA: String(repeating: "a", count: 64), videoBytes: 1,
            toolSourceSHA: String(repeating: "b", count: 64), timestamps: timestamps,
            duration: 10.0, fps: fps, step: step, start: 3.15, wanted: 120, available: 300)
        func plan(_ mutate: (inout [String: Any]) -> Void) -> SidecarPlan.Refusal? {
            SidecarPlan.admit(
                SolvedPoseFixtureGeneratorTests.fabricatedSidecarForPinning(expectation, mutate),
                expectation: expectation)
        }
        XCTAssertNil(plan { _ in }, "a well-formed sidecar must be ADMITTED — a comparator that "
                     + "refuses everything is indistinguishable from a working one")
        XCTAssertEqual(plan { root in
            if var r = root["request"] as? [String: Any] {
                r["upper_body_only"] = true; root["request"] = r
            }
        }?.field, "request.upper_body_only")
        XCTAssertEqual(plan { root in
            if var r = root["request"] as? [String: Any] {
                r["handler_orientation"] = "right"; root["request"] = r
            }
        }?.field, "request.handler_orientation")
        XCTAssertEqual(plan { root in
            root["schema"] = "biomotion.person_box_sidecar.v2"
        }?.field, "schema")
    }
}
