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
}
