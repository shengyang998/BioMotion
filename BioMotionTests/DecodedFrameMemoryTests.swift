import XCTest
import AVFoundation
import CoreVideo
import UIKit
@testable import BioMotion

/// What the decoded frames of one run cost in RAM, measured.
///
/// # The failure this is about
///
/// `OfflineSessionRunner.decodeFrames` decodes EVERY sampled timestamp into a
/// `UIImage` and appends them all to one array before any processing starts, and
/// each image is then retained by its `FrameResult` for the whole playback
/// session. So the peak is `frames × width × height × 4`, paid in full, on a
/// phone that is at the same time holding a 1.31 GiB Core ML model. Nothing
/// downstream can hand that memory back — which is why the bound has to live in
/// the decoder, and why it is a hard cap rather than advice.
///
/// The trigger is the app's own advice. `GaitLoadSummary.resolutionSentence`
/// tells the user "filming at 61 fps would resolve ±5%"; the user films at 60,
/// and the native window grows from 120 frames to 240. At 1080p that is 1.99 GB;
/// at 120 fps 3.98 GB; at 240 fps 4.98 GB. A 4K clip at plain 30 fps, which
/// nothing had considered at all, is 3.98 GB.
///
/// # What is measured here and what is arithmetic
///
/// MEASURED: the byte size of the bitmap the real `AVAssetImageGenerator`
/// actually hands back for a real clip, and the process's own
/// `phys_footprint` across a run of decodes. ARITHMETIC: multiplying that
/// per-frame size by the frame count from `FrameSource.sampleTimestamps`, which
/// is exact rather than an estimate because every frame of one clip comes back
/// at the same size — asserted, not assumed. The whole window is deliberately
/// NOT held at once: doing so would make this test allocate ~1 GB and a test
/// that can jetsam the runner takes the other 325 with it.
final class DecodedFrameMemoryTests: XCTestCase {

    // MARK: - The cap, over the whole rate × resolution matrix

    /// Every configuration the app can reach lands inside the budget, and the
    /// validated one is untouched.
    ///
    /// This is the arithmetic in one place: the cap is not a fixed pixel size,
    /// it is whatever keeps `frames × pixels × 4` under
    /// `decodedWindowBudgetBytes`, so filming faster costs spatial resolution
    /// instead of costing memory.
    func testEveryCaptureRateAndResolutionLandsInsideTheBudget() {
        let budget = Double(FrameSource.decodedWindowBudgetBytes)
        let resolutions: [(String, CGSize)] = [
            ("owner_576x1024", CGSize(width: 576, height: 1024)),
            ("owner_576x768", CGSize(width: 576, height: 768)),
            ("720p", CGSize(width: 1280, height: 720)),
            ("1080p", CGSize(width: 1920, height: 1080)),
            ("1080p_portrait", CGSize(width: 1080, height: 1920)),
            ("4K", CGSize(width: 3840, height: 2160)),
        ]
        for rate in [30.0, 60.0, 120.0, 240.0] {
            for (label, natural) in resolutions {
                let frames = FrameSource.worstCaseDecodedFrameCount(duration: 10,
                                                                    nominalFrameRate: rate)
                let cap = FrameSource.maximumDecodedSize(naturalSize: natural, duration: 10,
                                                         nominalFrameRate: rate)
                let out = FrameSource.decodedSize(naturalSize: natural,
                                                  cappedToSquareSide: cap.width)
                let peak = Double(frames) * Double(FrameSource.decodedFrameBytes(
                    width: Int(out.width), height: Int(out.height)))
                print("MEMORY-METRIC cap rate=\(rate) res=\(label) frames=\(frames) "
                      + "decoded=\(Int(out.width))x\(Int(out.height)) "
                      + "peak_MB=\(peak / 1e6) budget_MB=\(budget / 1e6)")
                XCTAssertLessThanOrEqual(peak, budget,
                                         "\(label) at \(rate) fps exceeds the decoded budget")
                XCTAssertLessThanOrEqual(out.width, natural.width, "\(label): never upscale")
                XCTAssertLessThanOrEqual(out.height, natural.height, "\(label): never upscale")
                // Aspect is preserved to within a pixel of rounding, so the
                // person box the pose model warps keeps its shape.
                XCTAssertEqual(out.width / out.height, natural.width / natural.height,
                               accuracy: 0.01, "\(label) at \(rate) fps")
            }
        }
    }

    /// **The non-regression that licenses the whole change.** The path that has
    /// actually been validated — 30 fps, up to 1080p, and every clip the product
    /// was built on — decodes at exactly its natural size. The cap only ever
    /// binds on configurations that were going to be terminated.
    /// ─── AMENDED 2026-08-21, sixteenth round ───
    /// The former name and claim — "the validated 30 fps path is decoded at FULL
    /// resolution" — was true at a 4 s window and is FALSE at 8 s for 1080p and
    /// above. This is a MEASURED CONSEQUENCE the window change's own
    /// pre-registration got WRONG: STATUS's round-16 header asserted "the decode
    /// budget does NOT bind", which was computed only against the owner's 576 px
    /// clips. The budget is defined as `maxFramesPerRun x 1920 x 1080 x 4`, so
    /// per-frame pixels are `budget / frames`: doubling the window from 120 to
    /// 240 frames HALVES the per-frame pixel allowance to 1,036,800.
    ///
    /// THE BUDGET IS NOT RAISED. It is a ~995 MB peak-memory ceiling and this
    /// project has no Release-device memory or runtime measurement at all;
    /// raising it would be inventing a number, which is exactly what
    /// `maxFramesPerRun`'s own registration refuses to do. What the window
    /// change did is push FURTHER along a trade `decodedWindowBudgetBytes`
    /// already declares in its own doc comment: spatial resolution for temporal
    /// resolution.
    ///
    /// THE PRECISE SCOPE, because "HD is downscaled" overstates it: at 30 fps
    /// and an 8 s window, 576x1024, 576x768 AND 1280x720 are all still decoded
    /// UNTOUCHED (589,824 / 442,368 / 921,600 px, all under 1,036,800). Only
    /// 1080p and above scale, and 1080p still lands ABOVE 720p. The consequence
    /// for the data plan is therefore that acquiring 1080p footage no longer
    /// buys a 1080p DECODE at this window — it buys roughly 1356x762 — not that
    /// higher-resolution acquisition stops helping.
    func testTheThirtyFpsPathIsUntouchedUpTo720pAndScalesAbove() {
        // Under the per-frame allowance: decoded exactly as they are.
        for natural in [CGSize(width: 576, height: 1024),   // owner's video_012 / 013
                        CGSize(width: 576, height: 768),    // owner's video_015
                        CGSize(width: 1280, height: 720)] {
            let cap = FrameSource.maximumDecodedSize(naturalSize: natural, duration: 10,
                                                     nominalFrameRate: 30)
            let out = FrameSource.decodedSize(naturalSize: natural,
                                              cappedToSquareSide: cap.width)
            XCTAssertEqual(out.width, natural.width, accuracy: 0.5,
                           "\(natural) must be untouched at 30 fps")
            XCTAssertEqual(out.height, natural.height, accuracy: 0.5)
        }

        // Over it: scaled, and the INVARIANTS are what is gated. The exact
        // side is a row-alignment artefact and is printed, not pinned.
        let frames = FrameSource.worstCaseDecodedFrameCount(duration: 10, nominalFrameRate: 30)
        for natural in [CGSize(width: 1920, height: 1080), CGSize(width: 1080, height: 1920)] {
            let cap = FrameSource.maximumDecodedSize(naturalSize: natural, duration: 10,
                                                     nominalFrameRate: 30)
            let out = FrameSource.decodedSize(naturalSize: natural,
                                              cappedToSquareSide: cap.width)
            let cost = frames * FrameSource.decodedFrameBytes(width: Int(out.width),
                                                              height: Int(out.height))
            print("DECODE-METRIC natural=\(natural) frames=\(frames) out=\(out) "
                  + "cost=\(cost) budget=\(FrameSource.decodedWindowBudgetBytes)")
            XCTAssertLessThanOrEqual(cost, FrameSource.decodedWindowBudgetBytes,
                                     "\(natural): a whole run must fit the budget")
            XCTAssertLessThanOrEqual(max(out.width, out.height), max(natural.width, natural.height),
                                     "\(natural): the cap never upscales")
            XCTAssertGreaterThan(max(out.width, out.height), 1280,
                                 "\(natural): 1080p acquisition must still beat a 720p decode, "
                                 + "or the input-quality lever is gone rather than dulled")
        }

        // And the budget IS that configuration, stated as arithmetic rather
        // than as a round number someone liked: 120 frames of 1080p.
        XCTAssertEqual(FrameSource.decodedWindowBudgetBytes,
                       FrameSource.maxFramesPerRun * 1920 * 1080
                           * FrameSource.bytesPerDecodedPixel)

        // The byte model is the one the decoder actually produces, padding
        // included — measured at 1357×763 on the real generator. Unchanged by
        // the window: this asserts the BYTE MODEL, not any window's output.
        XCTAssertEqual(FrameSource.decodedFrameBytes(width: 1357, height: 763), 4_150_720)
        XCTAssertEqual(FrameSource.decodedFrameBytes(width: 1920, height: 1080), 1920 * 1080 * 4,
                       "a 16-pixel-aligned width needs no padding")

        // The former portrait-1080p exception (1072x1907, an 8 px row-alignment
        // loss against a budget defined by the landscape case) is SUPERSEDED at
        // the 8 s window: portrait 1080p is now scaled by the pixel allowance
        // like the landscape case, and the alignment artefact is no longer the
        // binding term. It is covered by the invariant loop above.
    }

    /// A clip whose track metadata will not parse still gets a bound — smaller,
    /// not absent. A memory cap that fails open is not a cap.
    func testUnreadableTrackDimensionsStillProduceABound() {
        for natural in [CGSize.zero,
                        CGSize(width: -1, height: 100),
                        CGSize(width: CGFloat.nan, height: CGFloat.nan),
                        CGSize(width: CGFloat.infinity, height: 1080)] {
            let cap = FrameSource.maximumDecodedSize(naturalSize: natural, duration: 10,
                                                     nominalFrameRate: 240)
            XCTAssertTrue(cap.width.isFinite && cap.width > 0, "\(natural)")
            XCTAssertEqual(cap.width, cap.height, "the fallback box is square")
            // Worst case for an unknown aspect is 1:1, and the square must hold
            // the budget even then.
            let frames = FrameSource.worstCaseDecodedFrameCount(duration: 10,
                                                                nominalFrameRate: 240)
            let peak = frames * FrameSource.decodedFrameBytes(width: Int(cap.width),
                                                              height: Int(cap.height))
            XCTAssertLessThanOrEqual(peak, FrameSource.decodedWindowBudgetBytes)
        }
    }

    // MARK: - The real decoder, on a real file

    /// **The measurement.** A synthetic 1080p 60 fps clip — the exact
    /// configuration the app's own advice produces — decoded through the
    /// shipping `FrameSource.VideoDecoder`.
    ///
    /// The clip is a moving rectangle on a gradient. Nothing here is, or may be,
    /// footage of a person.
    func testASixtyFpsClipDecodesInsideTheBudgetAndTheFootprintAgrees() async throws {
        let natural = CGSize(width: 1920, height: 1080)
        let url = try Self.writeSyntheticClip(size: natural, fps: 60, frames: 243)
        defer { try? FileManager.default.removeItem(at: url) }

        let decoder = FrameSource.VideoDecoder(url: url)
        let duration = try await decoder.duration()
        let rate = await decoder.nominalFrameRate()
        let (timestamps, _) = FrameSource.sampleTimestamps(
            duration: duration,
            mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
            nominalFrameRate: rate)
        XCTAssertGreaterThan(timestamps.count, FrameSource.maxFramesPerRun,
                             "a 60 fps clip must ask for more frames than the 30 fps budget — "
                             + "that growth is the whole problem")

        // Every frame of one clip is the same size, so the peak is exact
        // arithmetic on a measured per-frame size. Sampled across the window
        // rather than assumed.
        var sizes = Set<Int>()
        var decodedPixelSize = CGSize.zero
        for i in [0, timestamps.count / 3, timestamps.count / 2, timestamps.count - 1] {
            let image = try await decoder.decodeFrame(at: timestamps[i])
            let cg = try XCTUnwrap(image.cgImage)
            sizes.insert(cg.bytesPerRow * cg.height)
            decodedPixelSize = CGSize(width: cg.width, height: cg.height)
        }
        XCTAssertEqual(sizes.count, 1, "frames of one clip must all decode to one size")
        let bytesPerFrame = try XCTUnwrap(sizes.first)
        let peak = bytesPerFrame * timestamps.count
        let uncapped = FrameSource.decodedFrameBytes(width: Int(natural.width),
                                                     height: Int(natural.height))
            * timestamps.count
        // The byte model used to size the cap must be the size the decoder
        // really produced, or the budget is bounding a number nothing pays.
        XCTAssertEqual(bytesPerFrame,
                       FrameSource.decodedFrameBytes(width: Int(decodedPixelSize.width),
                                                     height: Int(decodedPixelSize.height)),
                       "modelled and measured bitmap bytes must agree exactly")

        print("MEMORY-METRIC decode_1080p60 frames=\(timestamps.count) "
              + "decoded=\(Int(decodedPixelSize.width))x\(Int(decodedPixelSize.height)) "
              + "bytes_per_frame=\(bytesPerFrame) peak_MB=\(Double(peak) / 1e6) "
              + "uncapped_peak_MB=\(Double(uncapped) / 1e6) "
              + "budget_MB=\(Double(FrameSource.decodedWindowBudgetBytes) / 1e6)")

        XCTAssertGreaterThan(uncapped, FrameSource.decodedWindowBudgetBytes,
                             "without the cap this clip is over budget — otherwise this test "
                             + "proves nothing")
        XCTAssertLessThanOrEqual(peak, FrameSource.decodedWindowBudgetBytes,
                                 "the whole decoded window must fit the budget")
        XCTAssertLessThan(decodedPixelSize.width, natural.width,
                          "the cap must actually have bound here")
        XCTAssertEqual(decodedPixelSize.width / decodedPixelSize.height,
                       natural.width / natural.height, accuracy: 0.01,
                       "aspect preserved, so the warped person box keeps its shape")

        // And the accounting is not a fiction: hold a bounded run of frames and
        // watch the process's own footprint move by what the arithmetic says.
        let held = 60
        let before = Self.physFootprintBytes()
        var images: [UIImage] = []
        images.reserveCapacity(held)
        for i in 0..<held {
            images.append(try await decoder.decodeFrame(at: timestamps[i]))
        }
        let after = Self.physFootprintBytes()
        let expected = Double(bytesPerFrame * held)
        let observed = Double(after) - Double(before)
        print("MEMORY-METRIC footprint held=\(held) expected_MB=\(expected / 1e6) "
              + "observed_MB=\(observed / 1e6) ratio=\(observed / expected)")
        XCTAssertEqual(images.count, held)
        // Loose on purpose — this is a real process footprint with an allocator
        // and an autorelease pool in it, not a leak check. What it rules out is
        // the decoded frames costing an order of magnitude more (or the bitmaps
        // being lazy and the accounting above meaning nothing).
        XCTAssertGreaterThan(observed, 0.5 * expected,
                             "if the footprint barely moved, `bytesPerRow × height` is not "
                             + "what these images cost and the budget is measuring nothing")
        XCTAssertLessThan(observed, 3.0 * expected)
        images.removeAll()
    }

    // MARK: - Helpers

    private static func physFootprintBytes() -> UInt64 {
        var info = task_vm_info_data_t()
        var count = mach_msg_type_number_t(MemoryLayout<task_vm_info_data_t>.size
                                           / MemoryLayout<natural_t>.size)
        let result = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                task_info(mach_task_self_, task_flavor_t(TASK_VM_INFO), $0, &count)
            }
        }
        return result == KERN_SUCCESS ? UInt64(info.phys_footprint) : 0
    }

    /// A synthetic clip: a gradient with a moving rectangle. All-keyframe so the
    /// exact seeking `FrameSource.VideoDecoder` requires is cheap.
    private static func writeSyntheticClip(size: CGSize, fps: Int32, frames: Int) throws -> URL {
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("biomotion-decode-\(UUID().uuidString).mov")
        let writer = try AVAssetWriter(outputURL: url, fileType: .mov)
        let input = AVAssetWriterInput(mediaType: .video, outputSettings: [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: Int(size.width),
            AVVideoHeightKey: Int(size.height),
            AVVideoCompressionPropertiesKey: [
                AVVideoMaxKeyFrameIntervalKey: 1,
                AVVideoExpectedSourceFrameRateKey: Int(fps),
                AVVideoAverageBitRateKey: 6_000_000,
            ],
        ])
        input.expectsMediaDataInRealTime = false
        let adaptor = AVAssetWriterInputPixelBufferAdaptor(
            assetWriterInput: input,
            sourcePixelBufferAttributes: [
                kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32BGRA),
                kCVPixelBufferWidthKey as String: Int(size.width),
                kCVPixelBufferHeightKey as String: Int(size.height),
            ])
        writer.add(input)
        XCTAssertTrue(writer.startWriting(), "\(writer.error?.localizedDescription ?? "")")
        writer.startSession(atSourceTime: .zero)

        let w = Int(size.width), h = Int(size.height)
        for i in 0..<frames {
            while !input.isReadyForMoreMediaData { Thread.sleep(forTimeInterval: 0.002) }
            guard let pool = adaptor.pixelBufferPool else {
                throw NSError(domain: "DecodedFrameMemoryTests", code: 1)
            }
            var buffer: CVPixelBuffer?
            CVPixelBufferPoolCreatePixelBuffer(nil, pool, &buffer)
            guard let pixelBuffer = buffer else {
                throw NSError(domain: "DecodedFrameMemoryTests", code: 2)
            }
            CVPixelBufferLockBaseAddress(pixelBuffer, [])
            if let base = CVPixelBufferGetBaseAddress(pixelBuffer) {
                let stride = CVPixelBufferGetBytesPerRow(pixelBuffer)
                let bytes = base.assumingMemoryBound(to: UInt8.self)
                let barX = (i * 7) % max(1, w - 64)
                for y in 0..<h {
                    let row = bytes + y * stride
                    let shade = UInt8((y * 255) / max(1, h - 1))
                    memset(row, Int32(shade), stride)
                    // A moving bar, so successive frames are not identical and
                    // the encoder is not free to emit nothing.
                    memset(row + barX * 4, 255, 64 * 4)
                }
            }
            CVPixelBufferUnlockBaseAddress(pixelBuffer, [])
            let time = CMTime(value: CMTimeValue(i), timescale: CMTimeScale(fps))
            XCTAssertTrue(adaptor.append(pixelBuffer, withPresentationTime: time))
        }
        input.markAsFinished()

        let done = XCTestExpectation(description: "writer finished")
        writer.finishWriting { done.fulfill() }
        XCTWaiter().wait(for: [done], timeout: 120)
        XCTAssertEqual(writer.status, .completed,
                       "\(writer.error?.localizedDescription ?? "")")
        return url
    }
}
