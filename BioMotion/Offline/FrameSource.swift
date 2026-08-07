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
        /// The two cost the same — both are bounded by `maxFramesPerRun`, i.e.
        /// by the number of Core ML calls — but they buy different things, and
        /// only one of them can measure gait.
        ///
        /// A contact lasts 167-247 ms on the owner's clips. Sampled at 2 fps
        /// that is 0.3-0.5 samples: contact duration is not merely imprecise,
        /// it is unobservable. At the video's native 30 fps the same contact is
        /// 5-7 samples, which is what makes the left/right claim possible at
        /// all — and it is also what BOUNDS it, because the edges of a contact
        /// sampled `N` times are located to ±½ frame each. That quantisation is
        /// the binding limit on this whole product (`GaitResolution`), and the
        /// only lever on it is the capture frame rate.
        ///
        /// 4 s at 30 fps is 120 frames = exactly `maxFramesPerRun`, so the
        /// model-call budget is unchanged from the previous 60 s at 2 fps.
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
    /// clip. The pose model runs at roughly 1s/frame on-device (per this file
    /// set's task brief) — an accidental high-fps sample over a long clip must
    /// not silently turn into an hours-long run. Callers should surface
    /// `wasTruncated` to the user rather than truncating silently.
    static let maxFramesPerRun = 120

    /// Length of the native-rate analysis window. 4 s holds 5-7 running strides
    /// at the cadences measured on the owner's clips (606, 593 and 647 ms), and
    /// at 30 fps it is exactly `maxFramesPerRun` frames.
    static let analysisWindowSeconds: Double = 4.0

    /// Used when a video reports no usable nominal frame rate. Every clip the
    /// owner supplied is 30 fps; this only has to be a sane fallback, and it is
    /// reported to the caller so a wrong guess is visible rather than silent.
    static let assumedFrameRateWhenUnknown: Double = 30.0

    /// Rates outside this are not a camera, they are a corrupt track header.
    /// 240 fps is the fastest an iPhone records at full resolution.
    static let plausibleFrameRates: ClosedRange<Double> = 1.0...240.0

    // MARK: - Photo

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
            let wanted = min(maxFramesPerRun, max(1, Int(seconds / step)))
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
        private let asset: AVURLAsset
        private let generator: AVAssetImageGenerator
        private static let decodeQueue = DispatchQueue(label: "com.biomotion.offline.videodecode", qos: .userInitiated)

        init(url: URL) {
            self.url = url
            let asset = AVURLAsset(url: url)
            self.asset = asset
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
            self.generator = gen
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
            return try await withCheckedThrowingContinuation { continuation in
                Self.decodeQueue.async { [generator] in
                    do {
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
