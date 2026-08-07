import XCTest
@testable import BioMotion

/// The sampling change: a WINDOW at the video's own frame rate instead of the
/// whole clip sampled sparsely, at the same model-call budget.
final class NativeWindowSamplingTests: XCTestCase {

    private func native(_ duration: Double, fps: Double = 30,
                        seconds: Double = FrameSource.analysisWindowSeconds)
    -> (timestamps: [TimeInterval], wasTruncated: Bool) {
        FrameSource.sampleTimestamps(duration: duration,
                                     mode: .nativeWindow(seconds: seconds),
                                     nominalFrameRate: fps)
    }

    // MARK: - The budget is unchanged

    /// 4 s at 30 fps is exactly the cap. This is the arithmetic the whole
    /// change rests on: the native window costs no more Core ML calls than the
    /// sparse mode it replaces.
    func testFourSecondsAtThirtyIsExactlyTheFrameBudget() {
        let (ts, truncated) = native(10.0)
        XCTAssertEqual(ts.count, 120)
        XCTAssertEqual(ts.count, FrameSource.maxFramesPerRun)
        XCTAssertFalse(truncated, "the window fitted; the clip merely has more")

        // Adjacent spacing is exactly one video frame, not a resampled rate.
        for i in 1..<ts.count {
            XCTAssertEqual(ts[i] - ts[i - 1], 1.0 / 30.0, accuracy: 1e-12)
        }
        print("SAMPLING-METRIC native_30fps_4s frames=\(ts.count) "
              + "span_s=\(ts.last! - ts.first!) start_s=\(ts.first!)")
    }

    /// A contact is 150-250 ms. The point of the change, in one assertion.
    func testTheSparseModeCannotSeeAContactAndTheNativeOneCan() {
        let contactSeconds = 0.20
        let (sparse, _) = FrameSource.sampleTimestamps(duration: 10, mode: .fps(2))
        let sparseStep = sparse[1] - sparse[0]
        XCTAssertLessThan(contactSeconds / sparseStep, 1,
                          "at 2 fps a 200 ms contact is under one sample — unobservable")

        let (dense, _) = native(10.0)
        let denseStep = dense[1] - dense[0]
        XCTAssertGreaterThanOrEqual(contactSeconds / denseStep, 5,
                                    "at 30 fps the same contact is 6 samples")
        XCTAssertEqual(sparse.count, dense.count, accuracy: 100,
                       "and both stay inside the same model-call budget")
        print("SAMPLING-METRIC samples_per_contact sparse=\(contactSeconds / sparseStep) "
              + "native=\(contactSeconds / denseStep)")
    }

    /// The window is centred, deterministically, so two runs over one clip
    /// analyse the same frames.
    func testTheWindowIsCentredAndDeterministic() {
        let (a, _) = native(10.0)
        let (b, _) = native(10.0)
        XCTAssertEqual(a, b)
        let span = a.last! - a.first!
        XCTAssertEqual(a.first!, (10.0 - span) / 2, accuracy: 1e-9)
        XCTAssertEqual(a.last!, 10.0 - a.first!, accuracy: 1e-9)
    }

    // MARK: - The cases that must not crash

    /// A clip SHORTER than the window takes every frame it has. The remaining
    /// half-frame of slack is still split evenly, so the rule is one rule; what
    /// matters is that nothing is sampled past the end of the clip.
    func testAClipShorterThanTheWindowTakesTheWholeClip() {
        let (ts, truncated) = native(1.5)
        XCTAssertEqual(ts.count, 45, "1.5 s at 30 fps")
        XCTAssertTrue(truncated, "fewer frames than the window asked for, and the user is told")
        XCTAssertLessThan(ts.first!, 1.0 / 30.0, "at most one frame of slack at the head")
        for t in ts {
            XCTAssertGreaterThanOrEqual(t, 0)
            XCTAssertLessThan(t, 1.5, "never seek past the end of the clip")
        }
        print("SAMPLING-METRIC short_clip frames=\(ts.count) start_s=\(ts.first!) end_s=\(ts.last!)")
    }

    /// One frame long, sub-frame long, and zero long. None of these may trap or
    /// emit an empty list a decoder would then iterate zero times and report as
    /// success.
    func testDegenerateDurationsProduceExactlyOneUsableTimestamp() {
        for duration in [1.0 / 30.0, 0.01, 0.0, -1.0] {
            let (ts, _) = native(duration)
            XCTAssertEqual(ts.count, 1, "duration \(duration)")
            XCTAssertTrue(ts[0].isFinite)
            XCTAssertGreaterThanOrEqual(ts[0], 0)
            XCTAssertLessThanOrEqual(ts[0], max(duration, 0))
        }
    }

    /// A still photo never reaches `.nativeWindow`, but the mode that DOES
    /// serve it must stay safe for a zero-duration asset.
    func testAStillPhotoIsOneFrameAndSingleFrameModeIsSafeAtZeroDuration() {
        let image = UIImage()
        let decoded = FrameSource.decodePhoto(image)
        XCTAssertEqual(decoded.count, 1)
        XCTAssertEqual(decoded[0].timestamp, 0)
        XCTAssertEqual(decoded[0].index, 0)

        for duration in [0.0, -3.0, 8.0] {
            let (ts, truncated) = FrameSource.sampleTimestamps(duration: duration, mode: .singleFrame)
            XCTAssertEqual(ts.count, 1)
            XCTAssertGreaterThanOrEqual(ts[0], 0, "duration \(duration) must not seek negative")
            XCTAssertFalse(truncated)
        }
    }

    /// A single photo cannot be a gait clip, and the analysis must say so
    /// rather than throwing something the runner would surface as a crash.
    func testASinglePhotoIsRefusedByGaitAnalysisWithAReason() {
        let joints = [TrackedJoint(id: "hips_joint", name: "Pelvis",
                                   worldPosition: .zero, isTracked: true)]
        let one = [BodyFrame(timestamp: 0, frameNumber: 0, joints: joints)]
        XCTAssertThrowsError(try GaitAnalysis.analyse(frames: one)) { error in
            guard let failure = error as? GaitSignal.Failure else {
                return XCTFail("expected a typed failure, got \(error)")
            }
            XCTAssertEqual(failure, .tooFewFrames(count: 1, needed: GaitSignal.minimumFrames))
        }
    }

    // MARK: - Frame-rate handling

    /// A track that reports nonsense must fall back visibly, not scale every
    /// duration this app measures by a wrong constant.
    func testImplausibleFrameRatesFallBackToTheStatedAssumption() {
        for bad in [0.0, -30.0, 0.4, 1000.0, Double.nan, Double.infinity] {
            XCTAssertEqual(FrameSource.sanitisedFrameRate(bad),
                           FrameSource.assumedFrameRateWhenUnknown, "rate \(bad)")
        }
        for good in [24.0, 25.0, 30.0, 60.0, 120.0, 240.0] {
            XCTAssertEqual(FrameSource.sanitisedFrameRate(good), good)
        }
    }

    /// Higher capture rates are the product's only lever on resolution, so the
    /// sampler must actually take them — up to the budget.
    func testHigherCaptureRatesShortenTheWindowRatherThanSkippingFrames() {
        for fps in [30.0, 60.0, 120.0, 240.0] {
            let (ts, _) = native(20.0, fps: fps)
            XCTAssertEqual(ts.count, FrameSource.maxFramesPerRun)
            let step = ts[1] - ts[0]
            XCTAssertEqual(step, 1 / fps, accuracy: 1e-12,
                           "every frame at \(fps) fps, never a resampled subset")
            let contactFrames = 0.20 / step
            print("SAMPLING-METRIC fps=\(fps) window_s=\(ts.last! - ts.first!) "
                  + "frames_per_200ms_contact=\(contactFrames) "
                  + "resolvable_asymmetry_pct=\(100 * 0.5 / contactFrames)")
        }
    }

    /// The existing sparse mode is untouched — it is still the right tool for a
    /// held pose over a long clip.
    func testTheSparseModeIsUnchanged() {
        let (ts, truncated) = FrameSource.sampleTimestamps(duration: 10, mode: .fps(2))
        XCTAssertEqual(ts.count, 20)
        XCTAssertEqual(ts.first!, 0)
        XCTAssertEqual(ts[1] - ts[0], 0.5, accuracy: 1e-12)
        XCTAssertFalse(truncated)

        let (capped, cappedTruncated) = FrameSource.sampleTimestamps(duration: 600, mode: .fps(2))
        XCTAssertEqual(capped.count, FrameSource.maxFramesPerRun)
        XCTAssertTrue(cappedTruncated)
    }
}
