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

    // MARK: - Photo

    static func decodePhoto(_ image: UIImage) -> [DecodedFrame] {
        [DecodedFrame(image: image, timestamp: 0, index: 0)]
    }

    // MARK: - Video

    /// Timestamps to sample, in ascending order, without decoding anything.
    /// Returns `(timestamps, wasTruncated)` — `wasTruncated` is true if the
    /// requested mode/duration combination would have exceeded
    /// `maxFramesPerRun` and was capped.
    static func sampleTimestamps(duration: TimeInterval, mode: SamplingMode) -> (timestamps: [TimeInterval], wasTruncated: Bool) {
        switch mode {
        case .singleFrame:
            return ([duration / 2.0], false)
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
        }
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
