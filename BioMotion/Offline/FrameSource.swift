import UIKit
import AVFoundation

/// Decodes a picked photo or video into a sequence of upright `UIImage` frames
/// for `SAM3DPoseEstimator`. Deliberately has no `PhotosUI` dependency — picking
/// is `OfflineImportView`'s job; this type only turns "a photo" or "a video file
/// URL" into frames.
enum FrameSource {

    struct DecodedFrame {
        let image: UIImage
        /// Seconds, monotonically increasing within one decoded sequence. For a
        /// photo this is always 0. For video this is the actual sampled media
        /// time (not wall-clock decode time), so downstream SG-filter dt math
        /// reflects real motion timing.
        let timestamp: TimeInterval
        let index: Int
    }

    enum SamplingMode: Equatable {
        /// Decode exactly one frame (the temporal midpoint of a video; the only
        /// option for a photo).
        case singleFrame
        /// Sample at a fixed rate, starting at t=0.
        case fps(Double)
        /// Every frame the video actually has, over a WINDOW of `seconds`
        /// centred on the clip.
        ///
        /// # Why a short window at native rate beats a long clip sampled sparsely
        ///
        /// Both modes are bounded, but they deliberately do NOT cost the same.
        /// Sparse sampling stops at `maxFramesPerRun` (120 Core ML calls).
        /// Native-rate sampling stops at `maxNativeWindowFrames` (601 calls),
        /// the cap derived to preserve at least 2.5 s even at 240 fps. Only the
        /// native mode can resolve a contact; its processing cost rises with
        /// the video's frame rate and the selector discloses that trade.
        ///
        /// A contact lasts 167-247 ms on the owner's clips. Sampled at 2 fps
        /// that is 0.3-0.5 samples: contact duration is not merely imprecise,
        /// it is unobservable. At the video's native 30 fps the same contact is
        /// 5-7 samples, which is what makes the left/right claim possible at
        /// all — and it is also what BOUNDS it, because the edges of a contact
        /// sampled `N` times are located to ±½ frame each. That quantisation is
        /// a binding term on the published CONTACT-TIMING claim
        /// (`GaitResolution`), and the only camera lever on that term is the
        /// capture frame rate. It does not validate foot-support dynamics.
        ///
        /// At 30 fps only, 4 s is 120 frames — the same call count as the old
        /// sparse cap. Higher native rates use more calls, up to 601 at 240 fps.
        case nativeWindow(seconds: Double)
    }

    enum FrameSourceError: Error, LocalizedError {
        case zeroDuration
        case decodeFailed(String)

        var errorDescription: String? {
            switch self {
            case .zeroDuration: return "The selected video has no playable duration."
            case .decodeFailed(let detail): return "Couldn't decode a video frame: \(detail)"
            }
        }
    }

    /// Safety cap on how many timestamps `sampleTimestamps` will ever emit for one
    /// clip. No Release-device pose-model or end-to-end runtime has been
    /// measured, so the cap bounds work by Core ML call count rather than by an
    /// invented minutes estimate. Callers should surface `wasTruncated` to the
    /// user rather than truncating silently.
    static let maxFramesPerRun = 120

    /// Length of the native-rate analysis window. 4 s holds 5-7 running strides
    /// at the cadences measured on the owner's clips (606, 593 and 647 ms), and
    /// at 30 fps it is exactly `maxFramesPerRun` frames.
    static let analysisWindowSeconds: Double = 4.0

    /// Frame budget for `.nativeWindow` ONLY.
    ///
    /// # Why this is not `maxFramesPerRun`
    ///
    /// The gait-timing surface's frame-rate advice is "film at a higher frame
    /// rate". A
    /// FRAME budget turns that advice into a refusal: at 120 fps a 120-frame cap
    /// is 1.0 s of footage, which holds one contact per side, and the analysis
    /// then refuses as `.tooFewContacts` — from footage that is strictly better
    /// than the 30 fps clip that worked. Measured spans under the old cap:
    /// 30 fps 3.967 s, 60 fps 1.983 s, 120 fps 0.992 s, 240 fps 0.496 s.
    ///
    /// What actually has to be held constant is the analysed SPAN, because that
    /// is what holds strides. This budget is DERIVED, not chosen: it is exactly
    /// what `minimumAnalysisSeconds` costs at the fastest rate an iPhone
    /// records at, `+1` because a window of `N` samples spans `N−1` intervals.
    /// Resulting spans: 30 fps → 3.967 s, 60 → 3.983 s, 120 → 3.992 s,
    /// 240 → 2.500 s.
    ///
    /// It is a real, bounded cost: 601 Core ML calls are materially more work
    /// than 120. The only native-stage timing receipts are separate Debug iOS
    /// Simulator measurements (moving-input warm-start IK at ~6 mm/frame:
    /// 1567 ms/frame at 77.8 iterations,
    /// unchanged-marker warm IK 49 ms, and a 520×109 QP at 194.4 ms). They are
    /// not additive end-to-end timings and say nothing about Release-device
    /// runtime. `GaitTimingSummary.resolutionSentence` will not recommend a rate
    /// this budget cannot cover — see `highestAnalysableFrameRate`.
    static let maxNativeWindowFrames = Int(minimumAnalysisSeconds * plausibleFrameRates.upperBound) + 1

    /// Shortest analysed span that can still hold the minimum number of
    /// complete contacts per side. ~4 strides at the 0.647 s cadence measured
    /// on the owner's slowest clip, against a 3-contacts-per-side minimum.
    static let minimumAnalysisSeconds: Double = 2.5

    /// The highest capture rate at which the native window still spans
    /// `minimumAnalysisSeconds` — i.e. the highest rate this pipeline can
    /// honestly recommend. 240 fps at the current budget, by construction.
    static var highestAnalysableFrameRate: Double {
        Double(maxNativeWindowFrames - 1) / minimumAnalysisSeconds
    }

    /// Honest selector copy for the native-rate mode. Kept beside the budget
    /// arithmetic so the UI cannot again promise four seconds and sparse-mode
    /// cost after a frame-budget change makes either statement false.
    static var nativeWindowDisclosure: String {
        String(format: "Samples every frame over up to %.0f seconds from the middle of the clip, "
               + "capped at %d frames (%.1f seconds at %.0f fps). This makes a roughly 200 ms "
               + "foot contact measurable; processing time rises with the video's frame rate.",
               analysisWindowSeconds, maxNativeWindowFrames, minimumAnalysisSeconds,
               plausibleFrameRates.upperBound)
    }

    /// Used when a video reports no usable nominal frame rate. Every clip the
    /// owner supplied is 30 fps; this only has to be a sane fallback, and it is
    /// reported to the caller so a wrong guess is visible rather than silent.
    static let assumedFrameRateWhenUnknown: Double = 30.0

    /// Rates outside this are not a camera, they are a corrupt track header.
    /// 240 fps is the fastest an iPhone records at full resolution.
    static let plausibleFrameRates: ClosedRange<Double> = 1.0...240.0

    // MARK: - Decoded-frame memory

    /// Bytes one decoded pixel occupies. `AVAssetImageGenerator.copyCGImage`
    /// hands back a 32-bit-per-pixel bitmap; the tests measure
    /// `bytesPerRow × height` rather than trusting this, so it is only used to
    /// SIZE the budget, never to report it.
    static let bytesPerDecodedPixel = 4

    /// Decoded bitmaps pad each row up to a boundary, so a frame costs more than
    /// `width × height × 4`. MEASURED: a 1357×763 decode reports
    /// `bytesPerRow = 5440`, i.e. 1360 pixels — 3 columns of padding, 4,150,720
    /// bytes against the 4,141,564 the pixel count predicts. Small, and it was
    /// still enough to put a "budget-sized" window 0.08% over the budget, so the
    /// byte model accounts for it instead of the budget carrying a fudge factor.
    static let decodedRowAlignmentBytes = 64

    /// What one decoded frame of this size actually occupies, rows included.
    static func decodedFrameBytes(width: Int, height: Int) -> Int {
        guard width > 0, height > 0 else { return 0 }
        let row = width * bytesPerDecodedPixel
        let aligned = ((row + decodedRowAlignmentBytes - 1) / decodedRowAlignmentBytes)
            * decodedRowAlignmentBytes
        return aligned * height
    }

    /// The size `AVAssetImageGenerator.maximumSize` yields for `naturalSize`:
    /// fit inside the box preserving aspect, never enlarging, floored to whole
    /// pixels. Exposed so the budget arithmetic and the tests use one model of
    /// the generator's behaviour rather than two.
    static func decodedSize(naturalSize: CGSize, cappedToSquareSide side: CGFloat) -> CGSize {
        let w = naturalSize.width, h = naturalSize.height
        guard w > 0, h > 0, side > 0 else { return .zero }
        let scale = min(1.0, side / max(w, h))
        return CGSize(width: (w * scale).rounded(.down), height: (h * scale).rounded(.down))
    }

    /// How many bytes of decoded bitmap one run may hold at once.
    ///
    /// # Why this exists, and why it is a hard cap rather than advice
    ///
    /// Every sampled frame is decoded into a `UIImage` before any of them is
    /// processed, and each one is then retained by its `FrameResult` for the
    /// whole playback session — so the peak is `frames × width × height × 4`,
    /// paid in full, on a phone that is simultaneously holding a 1.31 GiB Core
    /// ML model. Nothing downstream can give that memory back, which is why the
    /// only place to bound it is here, at the decoder.
    ///
    /// # Why THIS number
    ///
    /// It is not a guess at a jetsam limit — that number is not measurable from
    /// this repo. It is the decoded cost of the configuration that already
    /// ships: `maxFramesPerRun` frames of 1080p. Consequences, all arithmetic:
    ///
    /// * 30 fps up to 1080p — the validated DECODE/MEMORY path, and every clip
    ///   the product
    ///   was built on (measured 576×1024 and 576×768) — is **unchanged**, because
    ///   its cost is already at or under the budget and the cap never upscales.
    /// * The one action the app recommends, filming faster, no longer multiplies
    ///   memory: 60/120/240 fps decode 240/480/601 frames and each is scaled to
    ///   land on this same peak (1080p → 1358×763, 959×539, 857×482) instead of
    ///   1.99/3.98/4.98 GB.
    /// * A 4K clip at 30 fps, which nothing previously bounded and which costs
    ///   3.98 GB, is scaled to exactly 1920×1080.
    ///
    /// # What it costs, stated rather than hidden
    ///
    /// Above 30 fps the subject's pixel height falls, and the pose model's own
    /// preprocessing warps the padded person box to a fixed 512 px square
    /// (`SAM3DPoseEstimator.PreprocessingConstants`), so below a box side of 512
    /// that warp is upsampling. Measured on the owner's three clips the box side
    /// is 360-711 px, i.e. the pipeline already both up- and downsamples there.
    /// This trades spatial resolution for temporal resolution — which is the
    /// trade the contact-timing grid calls for. It does not make dynamics
    /// available: the bundled models still lack validated foot support. It is
    /// still a trade, and it is the reason this constant is one line.
    static let decodedWindowBudgetBytes =
        maxFramesPerRun * 1920 * 1080 * bytesPerDecodedPixel

    /// The most frames any sampling mode could ask this clip for.
    ///
    /// The decoder is handed one timestamp at a time and never learns which mode
    /// produced them, so the cap has to be sized for the worst case: the native
    /// window at this clip's own rate, or `maxFramesPerRun`, whichever is larger
    /// (no other mode can exceed the latter — see `sampleTimestamps`).
    static func worstCaseDecodedFrameCount(duration: TimeInterval,
                                           nominalFrameRate: Double) -> Int {
        let native = sampleTimestamps(duration: duration,
                                      mode: .nativeWindow(seconds: analysisWindowSeconds),
                                      nominalFrameRate: nominalFrameRate).timestamps.count
        return max(1, max(native, maxFramesPerRun))
    }

    /// The `AVAssetImageGenerator.maximumSize` that keeps a whole run inside
    /// `decodedWindowBudgetBytes`, as a SQUARE box.
    ///
    /// Square on purpose. `maximumSize` fits the image inside the box preserving
    /// aspect, so a square of side `S` caps the LONG side at `S` whichever way
    /// the clip is rotated — and `appliesPreferredTrackTransform` means the
    /// generator's output orientation need not match `naturalSize`. Passing a
    /// non-square box would bind on the wrong axis for a portrait clip.
    ///
    /// Never upscales: the scale is clamped at 1, so a clip already inside the
    /// budget is decoded exactly as it is today.
    static func maximumDecodedSize(naturalSize: CGSize,
                                   duration: TimeInterval,
                                   nominalFrameRate: Double) -> CGSize {
        let frames = worstCaseDecodedFrameCount(duration: duration,
                                                nominalFrameRate: nominalFrameRate)
        let pixelBudget = Double(decodedWindowBudgetBytes)
            / Double(frames) / Double(bytesPerDecodedPixel)
        let w = Double(naturalSize.width), h = Double(naturalSize.height)
        guard w.isFinite, h.isFinite, w > 0, h > 0 else {
            // Track dimensions unreadable. Fall back to the box that holds the
            // budget for ANY aspect ratio — the worst case is 1:1, so a square
            // of side √budget is safe without knowing the shape. Shrunk by the
            // row-alignment allowance for the same reason as below.
            let square = pixelBudget.squareRoot().rounded(.down)
            let side = Self.largestSideFittingBudget(
                naturalSize: CGSize(width: square, height: square),
                from: square, frames: frames)
            return CGSize(width: side, height: side)
        }
        // Pixel-count estimate first, then step down until the ALIGNED byte cost
        // fits. The correction is a fraction of a percent, so this settles in a
        // few steps, and it runs once per clip.
        let scale = min(1.0, (pixelBudget / (w * h)).squareRoot())
        let side = Self.largestSideFittingBudget(naturalSize: naturalSize,
                                                 from: (max(w, h) * scale).rounded(.down),
                                                 frames: frames)
        return CGSize(width: side, height: side)
    }

    private static func largestSideFittingBudget(naturalSize: CGSize,
                                                 from start: Double,
                                                 frames: Int) -> CGFloat {
        var side = CGFloat(max(1.0, start))
        while side > 1 {
            let out = decodedSize(naturalSize: naturalSize, cappedToSquareSide: side)
            let cost = frames * decodedFrameBytes(width: Int(out.width), height: Int(out.height))
            if cost <= decodedWindowBudgetBytes { break }
            side -= 1
        }
        return side
    }

    /// `maximumDecodedSize` for a clip on disk, reading the three facts it needs
    /// from the container. Every read is optional: a clip whose track metadata
    /// cannot be parsed still decodes, under the aspect-agnostic fallback, which
    /// is smaller rather than larger. Never throws — a memory bound that can
    /// fail open is not a bound.
    static func maximumDecodedSize(forVideoAt url: URL) async -> CGSize {
        let asset = AVURLAsset(url: url)
        let duration = (try? await asset.load(.duration))?.seconds ?? 0
        var rate = assumedFrameRateWhenUnknown
        var natural = CGSize.zero
        if let track = try? await asset.loadTracks(withMediaType: .video).first {
            if let r = try? await track.load(.nominalFrameRate) {
                rate = sanitisedFrameRate(Double(r))
            }
            if let s = try? await track.load(.naturalSize) {
                natural = s
            }
        }
        return maximumDecodedSize(naturalSize: natural,
                                  duration: duration.isFinite ? duration : 0,
                                  nominalFrameRate: rate)
    }

    // MARK: - Photo

    /// A single decoded frame, at whatever size the picker handed over.
    ///
    /// Deliberately NOT capped by `decodedWindowBudgetBytes`: one image is one
    /// image, and the budget exists to bound a `frames × pixels` product that a
    /// photo does not have. The video path is the one that multiplies.
    static func decodePhoto(_ image: UIImage) -> [DecodedFrame] {
        [DecodedFrame(image: image, timestamp: 0, index: 0)]
    }

    // MARK: - Video

    /// Timestamps to sample, in ascending order, without decoding anything.
    /// Returns `(timestamps, wasTruncated)` — `wasTruncated` is true if fewer
    /// samples came back than the mode asked for, whether because
    /// `maxFramesPerRun` capped it or because the clip is shorter than the
    /// requested window.
    ///
    /// - Parameter nominalFrameRate: the video track's own frame rate, needed
    ///   only by `.nativeWindow`. Ignored by the other modes.
    static func sampleTimestamps(duration: TimeInterval,
                                 mode: SamplingMode,
                                 nominalFrameRate: Double = assumedFrameRateWhenUnknown) -> (timestamps: [TimeInterval], wasTruncated: Bool) {
        switch mode {
        case .singleFrame:
            return ([max(duration, 0) / 2.0], false)

        case .fps(let fps):
            guard fps > 0, duration > 0 else { return ([0], false) }
            let step = 1.0 / fps
            var t = 0.0
            var out: [TimeInterval] = []
            while t < duration && out.count < maxFramesPerRun {
                out.append(t)
                t += step
            }
            if out.isEmpty { out = [0] }
            let wasTruncated = out.count >= maxFramesPerRun && (duration / step) > Double(maxFramesPerRun)
            return (out, wasTruncated)

        case .nativeWindow(let seconds):
            let fps = sanitisedFrameRate(nominalFrameRate)
            guard duration > 0, seconds > 0 else { return ([0], false) }
            let step = 1.0 / fps
            // How many samples the WINDOW wants, and how many the CLIP has.
            // `floor` on both, because a sample at exactly `duration` is past
            // the last frame.
            let wanted = min(maxNativeWindowFrames, max(1, Int(seconds / step)))
            let available = max(1, Int(duration / step))
            let count = min(wanted, available)
            // Centre the window. A running clip's usable stretch is far more
            // often in the middle than at the very start, where the subject is
            // still entering frame — and centring is deterministic, so two runs
            // over one clip analyse the same frames.
            let span = Double(count - 1) * step
            let start = max(0, (duration - span) / 2)
            let timestamps = (0..<count).map { start + Double($0) * step }
            // Truncated means "you got fewer samples than the window asked
            // for". Both causes matter to the user and both are actionable —
            // one says film longer, the other says the clip is long enough that
            // only part of it was analysed.
            let requested = min(Int(seconds / step), Int(duration / step))
            return (timestamps, count < max(requested, Int(seconds / step)))
        }
    }

    /// Clamps a track's reported frame rate into something a camera could have
    /// produced, falling back when it is 0 or non-finite (which
    /// `AVAssetTrack.nominalFrameRate` genuinely returns for some containers).
    static func sanitisedFrameRate(_ raw: Double) -> Double {
        guard raw.isFinite, plausibleFrameRates.contains(raw) else {
            return assumedFrameRateWhenUnknown
        }
        return raw
    }

    final class VideoDecoder {
        let url: URL
        /// Production decoders retain the app-owned copy for every asset read.
        /// Fixture tests may still construct a decoder from a stable URL.
        private let video: AppOwnedTemporaryVideo?
        private let asset: AVURLAsset
        private let generator: AVAssetImageGenerator
        private static let decodeQueue = DispatchQueue(label: "com.biomotion.offline.videodecode", qos: .userInitiated)

        /// The decode-size cap for this clip, resolved once.
        ///
        /// Started in `init`, so it is a plain `let` — no lazy field to race on,
        /// and no assumption that the caller decodes serially. A production
        /// task captures the video owner through completion because this
        /// unstructured task can outlive the decoder itself. The value is
        /// applied to the generator on `decodeQueue`, which is the only place
        /// the generator is touched at all.
        private let sizeCap: Task<CGSize, Never>

        init(video: AppOwnedTemporaryVideo) {
            self.video = video
            self.url = video.url
            let asset = AVURLAsset(url: video.url)
            self.asset = asset
            self.sizeCap = Task { [video] in
                let cap = await FrameSource.maximumDecodedSize(forVideoAt: video.url)
                withExtendedLifetime(video) {}
                return cap
            }
            let gen = Self.makeGenerator(asset: asset)
            self.generator = gen
        }

        /// URL-only construction is reserved for tests whose fixture lifecycle
        /// is controlled by the test itself. Picker-backed production paths use
        /// `init(video:)` so the private copy cannot disappear during decode.
        init(url: URL) {
            self.video = nil
            self.url = url
            let asset = AVURLAsset(url: url)
            self.asset = asset
            self.sizeCap = Task { await FrameSource.maximumDecodedSize(forVideoAt: url) }
            let gen = Self.makeGenerator(asset: asset)
            self.generator = gen
        }

        private static func makeGenerator(asset: AVURLAsset) -> AVAssetImageGenerator {
            let gen = AVAssetImageGenerator(asset: asset)
            // Pixel data comes out already upright, matching how the video plays
            // back — SAM3DPoseEstimator/Vision then just work in `UIImage.size`
            // space with no further orientation handling needed.
            gen.appliesPreferredTrackTransform = true
            // Exact (not keyframe-snapped) seeking. This clip is already sampled
            // sparsely (2fps default), so trading a little decode speed for
            // sampling the frame we actually asked for is the right tradeoff for
            // a biomechanics tool where frame timing matters.
            gen.requestedTimeToleranceBefore = .zero
            gen.requestedTimeToleranceAfter = .zero
            return gen
        }

        func duration() async throws -> TimeInterval {
            let d = try await asset.load(.duration)
            guard d.isNumeric, d.seconds > 0 else { throw FrameSourceError.zeroDuration }
            return d.seconds
        }

        /// The video track's own frame rate, clamped to something plausible.
        ///
        /// Never throws: a clip whose track metadata cannot be read is still
        /// analysable, it just gets `assumedFrameRateWhenUnknown`. The caller
        /// surfaces which one it used, because a wrong rate would silently
        /// rescale every duration this app measures.
        func nominalFrameRate() async -> Double {
            guard let track = try? await asset.loadTracks(withMediaType: .video).first,
                  let rate = try? await track.load(.nominalFrameRate) else {
                return assumedFrameRateWhenUnknown
            }
            return sanitisedFrameRate(Double(rate))
        }

        /// Decodes exactly one frame at `timestamp` seconds. Throws on failure —
        /// callers treat this as a single-frame failure, not a whole-run abort.
        /// Uses the long-stable synchronous `copyCGImage(at:actualTime:)` API
        /// wrapped in our own background queue + continuation, rather than a
        /// newer async convenience wrapper this file's author could not verify
        /// the exact signature of without a build.
        func decodeFrame(at timestamp: TimeInterval) async throws -> UIImage {
            let time = CMTime(seconds: timestamp, preferredTimescale: 600)
            // Bounded BEFORE the first decode, not after: the whole point is
            // that the full-resolution bitmap is never materialised. See
            // `FrameSource.decodedWindowBudgetBytes`.
            let cap = await sizeCap.value
            return try await withCheckedThrowingContinuation { continuation in
                Self.decodeQueue.async { [generator] in
                    do {
                        if generator.maximumSize != cap { generator.maximumSize = cap }
                        let cgImage = try generator.copyCGImage(at: time, actualTime: nil)
                        continuation.resume(returning: UIImage(cgImage: cgImage))
                    } catch {
                        continuation.resume(throwing: FrameSourceError.decodeFailed(error.localizedDescription))
                    }
                }
            }
        }
    }
}
