import CoreML
import Vision
import UIKit
import simd

/// Loads the frozen `SAM3DBodyPose` Core ML model and runs the preprocessing /
/// postprocessing pipeline described in the frozen interface contract that this
/// file was written against:
///
///   image   : [1,3,512,384] Float16, RGB, (x/255-mean)/std, mean=[0.485,0.456,0.406]
///             std=[0.229,0.224,0.225], from affine-warping the padded person bbox
///             to 512x512 then cropping width [64:-64].
///   ray_map : [1,2,512,384] Float16, per-pixel camera ray field, same crop geometry.
///   cliff   : [1,3] Float16, [(cx-cx_int)/f, (cy-cy_int)/f, b/f], where `b`
///             is the padded square crop side from the frozen contract.
///   joint_coords/global_rots/cam_t/keypoints_2d outputs, Float32, per contract.
///
/// The exact contract revision is pinned by
/// `BioMotion/Resources/SAM3DBodyPose.lock.json`. Every interface field and
/// geometric formula below has been cross-checked against it. The formulas were
/// also independently derived from the released Python source and verified on
/// concrete numeric examples with its real transform functions; see the inline
/// citations and `BioMotion/Offline/README.md` for the evidence and residual risk.
final class SAM3DPoseEstimator {

    // MARK: - Output

    /// One frame's worth of raw model output, in the model's native units/frame
    /// exactly as the frozen contract specifies: meters, MHR-native frame,
    /// UN-flipped (no `[..., [1,2]] *= -1`). No axis transform is applied here —
    /// that is `MHRRetarget`'s job (see its extensive verification that MHR-native
    /// is already X-right/Y-up/Z-toward-camera, matching ARKit).
    struct Output {
        /// 127 MHR joint positions, meters, MHR-native frame. Consumed directly by
        /// `MHRRetarget.makeBodyFrame(jointCoords:timestamp:frameNumber:)`.
        let jointCoords: [SIMD3<Float>]
        /// 127 per-joint global rotation matrices, same frame as `jointCoords`.
        /// `matrix * vector` matches PyTorch's row-major `R @ v` — see
        /// `readMat3x3Array`. Not currently consumed by MHRRetarget's given
        /// signature; exposed here for fidelity to the contract and future use.
        let globalRots: [simd_float3x3]
        /// Camera-relative translation in metres: OpenCV-style X-right,
        /// Y-down and positive Z away from the camera.
        let camT: SIMD3<Float>
        /// 70 2D keypoints in the 512x384 crop pixel frame (x, y).
        let keypoints2D: [SIMD2<Float>]
        /// True if Vision found no person and preprocessing fell back to the
        /// whole image as the "bbox" — matches the Python path's own fallback
        /// (`sam_3d_body_estimator.py:125`, `boxes = [0,0,width,height]` when no
        /// detector is configured). The estimate still runs. Admission is
        /// source-specific downstream: a photo fallback remains analysable,
        /// while a video fallback is review-only and splits temporal analysis.
        let usedFallbackBBox: Bool

        /// Checksum of the normalised `image` tensor actually handed to Core ML,
        /// and of the returned `joint_coords`.
        ///
        /// These were built to test the hypothesis that the two Core ML backends
        /// compute different things from identical bytes, after the Mac and the
        /// phone disagreed about a sprinter's recovery leg. **That hypothesis
        /// was wrong.** The cause was `upperBodyOnly` (see
        /// `makePersonRectangleRequest`): the phone detected its own torso-only
        /// person box while the Mac reproduction was handed a full-body one, so
        /// the two were never given the same crop. The Mac's much-cited
        /// invariance to person-box changes was the clue, not the mystery — all
        /// the probed boxes were generous ones.
        ///
        /// They stay because the property they check is still worth checking:
        /// matching input checksums with differing output checksums would mean
        /// the backends genuinely diverge. Read them as a parity check, not as
        /// evidence about any open bug.
        ///
        /// Both are order-sensitive FNV-1a over the raw bit patterns, so a
        /// single flipped mantissa bit changes them.
        let inputChecksum: UInt64
        let outputChecksum: UInt64
        /// Stage fingerprints upstream of the model, so a divergence localises
        /// without a round trip per stage: decoded source pixels, the person box
        /// (centre + square side), and the warped crop before normalisation.
        let sourceHash: UInt64
        let bboxHash: UInt64
        let warpHash: UInt64
    }

    /// FNV-1a over the bit patterns of a Float sequence. Order-sensitive and
    /// exact — no tolerance — because the question is whether two backends were
    /// given, and produced, literally the same bits.
    static func checksum<S: Sequence>(_ values: S) -> UInt64 where S.Element == Float {
        var h: UInt64 = 0xcbf29ce484222325
        for v in values {
            var bits = UInt64(v.bitPattern)
            for _ in 0..<4 {
                h = (h ^ (bits & 0xff)) &* 0x100000001b3
                bits >>= 8
            }
        }
        return h
    }

    /// FNV-1a over raw bytes — for image buffers, where the question is whether
    /// two machines produced literally the same pixels.
    static func checksumBytes(_ bytes: [UInt8]) -> UInt64 {
        var h: UInt64 = 0xcbf29ce484222325
        for b in bytes { h = (h ^ UInt64(b)) &* 0x100000001b3 }
        return h
    }

    /// The decoded source frame's pixels, in a layout independent of row
    /// padding, so the hash reflects the image rather than the allocation.
    static func sourcePixels(_ image: UIImage) -> [UInt8] {
        guard let cg = image.cgImage else { return [] }
        let w = cg.width, h = cg.height
        var buf = [UInt8](repeating: 0, count: w * h * 4)
        buf.withUnsafeMutableBytes { raw in
            guard let ctx = CGContext(data: raw.baseAddress, width: w, height: h,
                                      bitsPerComponent: 8, bytesPerRow: w * 4,
                                      space: CGColorSpaceCreateDeviceRGB(),
                                      bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else { return }
            ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
        }
        return buf
    }

    enum EstimatorError: LocalizedError {
        case imageDecodeFailed
        case modelLoadFailed(Error)
        case preprocessingFailed(String)
        case predictionFailed(Error)
        case missingOutputFeature(String)
        case unexpectedOutputDataType(String)
        case unexpectedOutputShape(String)
        case invalidOutputValue(String)

        var errorDescription: String? {
            switch self {
            case .imageDecodeFailed:
                return "Couldn't read pixel data from the selected image."
            case .modelLoadFailed(let error):
                return "Pose model failed to load: \(error.localizedDescription)"
            case .preprocessingFailed(let detail):
                return "Preprocessing failed: \(detail)"
            case .predictionFailed(let error):
                return "Pose model prediction failed: \(error.localizedDescription)"
            case .missingOutputFeature(let name):
                return "Pose model output is missing expected field \"\(name)\" — check the compiled SAM3DBodyPose artifact against the frozen lock."
            case .unexpectedOutputDataType(let detail):
                return "Pose model output data type mismatch: \(detail)"
            case .unexpectedOutputShape(let detail):
                return "Pose model output shape mismatch: \(detail)"
            case .invalidOutputValue(let detail):
                return "Pose model returned an invalid value: \(detail)"
            }
        }
    }

    // MARK: - Preprocessing constants
    //
    // Single source of truth for every magic number in this file, so a
    // CONTRACT.md cross-check only has to touch this block. All formulas below
    // that use these constants were empirically verified against the actual
    // Python transforms (see file header) with a synthetic 800x600 image / bbox:
    // bbox_center=(400,330), padded+square side=625, affine scale=0.8192,
    // tx=-71.68, ty=-14.336, cam_int f=1000/cx=400/cy=300, ray_map matched the
    // library's `get_ray_condition` output exactly at 5 sampled pixels incl. both
    // corners and the center.
    //
    // That example uses a TALL bbox, for which `max(bw/0.75, bh)` and
    // `max(bw, bh)` return the same side — so it cannot validate the crop
    // formula. Any future check must include a bbox WIDER than 3:4 (plank,
    // lying stretch, wide-stance squat, arms outstretched), where the two
    // disagree by up to 33%. Verified against the real
    // `bbox_xyxy2cs` + `fix_aspect_ratio` pair on 4 boxes incl. 600x200: exact.
    enum PreprocessingConstants {
        /// Model input spatial size (H, W). H is untouched by the width crop;
        /// W is the post-crop width (squareWarpSize - 2*widthCropMargin).
        static let inputHeight = 512
        static let inputWidth = 384
        /// Side of the square the padded bbox is warped to, BEFORE the width crop.
        static let squareWarpSize = 512
        /// Columns dropped from each side of the 512-wide square warp to reach
        /// 384 — matches the Python reference's `vit_hmr_512_384` backbone path
        /// (`base_model.py` `data_preprocess`: `batch_inputs[:, :, :, 64:-64]`,
        /// applied identically to `ray_cond` in `sam3d_body.py`'s
        /// `forward_pose_branch`).
        static let widthCropMargin = 64
        /// `GetBBoxCenterScale`'s padding factor, as constructed by
        /// `SAM3DBodyEstimator.transform` (bbox_utils.py / common.py default 1.25).
        static let bboxPadding: CGFloat = 1.25
        /// The 3:4 prior `TopdownAffine` expands the padded box to before it
        /// squares it (`bbox_utils.py` `fix_aspect_ratio(scale, 0.75)`).
        static let bboxPriorAspect: CGFloat = 0.75
        static let imageMean: [Float] = [0.485, 0.456, 0.406]
        static let imageStd: [Float] = [0.229, 0.224, 0.225]
        static let numBodyJoints = 127
        static let numOutputKeypoints2D = 70
    }

    // MARK: - State

    private var model: MLModel?
    private(set) var isLoaded = false
    private static let workQueue = DispatchQueue(label: "com.biomotion.sam3d.coreml", qos: .userInitiated)

    // MARK: - Loading

    /// Observable download state for the Background Assets pack that carries the
    /// weights. The import view reads this live state; a thrown availability
    /// error is only the runner's prompt non-blocking control-flow signal.
    @MainActor static var modelStore: AssetPackModelStore { AssetPackModelStore.shared }

    /// An estimator is constructed when the import screen appears, several user
    /// actions before the model is actually needed (pick a clip, choose a
    /// sampling rate, tap Run). Spending that time starting the asset-pack
    /// download is free: the pack's manifest already marks it `prefetch`, so the
    /// OS was going to fetch it after install anyway — this only removes the
    /// case where the very first Run tap is the thing that starts a 1.31 GiB
    /// transfer. Fire-and-forget; failures surface later through
    /// `loadModelIfNeeded()`.
    init() {
        Task { await AssetPackModelStore.shared.beginPrefetch() }
    }

    /// Idempotent — safe to call before every `estimate(uiImage:)`.
    ///
    /// The 1.31 GiB model is no longer in the app bundle; it arrives as an
    /// Apple-Hosted Managed Background Assets pack. `AssetPackModelStore`
    /// resolves it (preferring a bundled developer copy when one exists) and,
    /// when the pack has not arrived yet, throws promptly while the observed
    /// store publishes checking, system progress, pause, retry and ready. This
    /// call never blocks on the transfer.
    func loadModelIfNeeded() async throws {
        if model != nil { return }
        let url = try await AssetPackModelStore.shared.resolveCompiledModelURL()
        let config = MLModelConfiguration()
        // computeUnits = .cpuAndGPU, deliberately NOT .all: ANE compilation of a
        // comparable model has previously cost ~10 minutes of cold load on this
        // device class (see this file's task brief). GPU is the accepted tradeoff.
        config.computeUnits = .cpuAndGPU
        let loaded = try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<MLModel, Error>) in
            Self.workQueue.async {
                do {
                    let m = try MLModel(contentsOf: url, configuration: config)
                    continuation.resume(returning: m)
                } catch {
                    continuation.resume(throwing: EstimatorError.modelLoadFailed(error))
                }
            }
        }
        model = loaded
        isLoaded = true
    }

    // MARK: - Estimate

    /// - Parameter personBoxNormalizedBottomLeft: an EXTERNALLY supplied person
    ///   box in Vision's own normalized, bottom-left-origin, Y-up space. `nil`
    ///   — the production default and the only value production passes
    ///   (`OfflineSessionRunner.swift:1079`) — is today's behaviour byte for
    ///   byte: the internal Vision call runs. A non-`nil` value skips Vision
    ///   entirely and runs the SAME shared conversion, so no caller can bypass
    ///   the flip, the bounds intersection or the degeneracy guard.
    ///
    ///   Registered 2026-08-14 (person-box sidecar amendment). It exists because
    ///   `VNDetectHumanRectanglesRequest.perform` THROWS on every frame inside
    ///   the iOS Simulator (`com.apple.Vision Code=9`, 12/12 measured), so the
    ///   fixture generator cannot obtain a box in the only host it may run in.
    ///   A box that arrives this way carries macOS-Vision provenance, which is
    ///   NOT iOS-Vision provenance; the fixture header records that as
    ///   `bbox_source macos_vision INTERIM`.
    func estimate(uiImage: UIImage,
                  personBoxNormalizedBottomLeft: CGRect? = nil) async throws -> Output {
        let model = try await ensureModelLoaded()

        guard uiImage.size.width > 0, uiImage.size.height > 0 else {
            throw EstimatorError.imageDecodeFailed
        }

        let (bboxRect, usedFallback) = try Self.resolvePersonBox(
            uiImage: uiImage, injectedNormalizedBottomLeft: personBoxNormalizedBottomLeft)
        let geometry = Self.computeCropGeometry(bbox: bboxRect, imageSize: uiImage.size)

        // Default (no real calibration, no FOV estimator) camera intrinsics —
        // matches `prepare_batch.py`'s fallback EXACTLY when neither `cam_int` nor
        // a FOV estimator is supplied, which is the code path this device runs
        // (no MoGe2-class model is bundled): f = sqrt(h²+w²), principal point at
        // image center. Verified empirically to make the `USE_INTRIN_CENTER`
        // branch of `_get_decoder_condition` numerically identical to the default
        // branch, since cam_int's principal point equals img_size/2 exactly here.
        let origW = Float(uiImage.size.width)
        let origH = Float(uiImage.size.height)
        let focalLength = (origW * origW + origH * origH).squareRoot()
        let camIntCx = origW / 2
        let camIntCy = origH / 2

        guard let rgba = Self.renderWarpedRGBA(source: uiImage, geometry: geometry) else {
            throw EstimatorError.preprocessingFailed("could not render the affine-warped square crop")
        }

        // Stage fingerprints, so a divergence localises in ONE reading instead of
        // a round trip per stage. The per-frame input checksum alone cannot say
        // whether two machines decoded the video differently, detected a
        // different person box, or warped differently — only that something
        // upstream of the model differs.
        let sourceHash = Self.checksumBytes(Self.sourcePixels(uiImage))
        let bboxHash = Self.checksum([Float(geometry.bboxCenter.x), Float(geometry.bboxCenter.y),
                                      Float(geometry.side)])
        let warpHash = Self.checksumBytes(rgba)

        let imageArray = try Self.makeImageTensor(rgba: rgba)
        let rayArray = try Self.makeRayMapTensor(geometry: geometry, focalLength: focalLength,
                                                  camIntCx: camIntCx, camIntCy: camIntCy)
        let cliffArray = try Self.makeCliffTensor(geometry: geometry, focalLength: focalLength,
                                                   camIntCx: camIntCx, camIntCy: camIntCy)

        let provider = try MLDictionaryFeatureProvider(dictionary: [
            "image": MLFeatureValue(multiArray: imageArray),
            "ray_map": MLFeatureValue(multiArray: rayArray),
            "cliff": MLFeatureValue(multiArray: cliffArray),
        ])

        // Checksum the exact bytes handed to Core ML. Float16 is widened to
        // Float first so the hash is comparable with a Mac-side reference that
        // holds the same values in single precision.
        let imgPtr = imageArray.dataPointer.bindMemory(to: Float16.self,
                                                       capacity: imageArray.count)
        let inputChecksum = Self.checksum(
            (0..<imageArray.count).lazy.map { Float(imgPtr[$0]) })

        let outputProvider = try await Self.runPrediction(model: model, input: provider)
        return try Self.parseOutput(outputProvider, usedFallbackBBox: usedFallback,
                                    inputChecksum: inputChecksum,
                                    sourceHash: sourceHash, bboxHash: bboxHash, warpHash: warpHash)
    }

    private func ensureModelLoaded() async throws -> MLModel {
        if let model { return model }
        try await loadModelIfNeeded()
        guard let model else {
            throw EstimatorError.modelLoadFailed(NSError(domain: "SAM3DPoseEstimator", code: -1,
                                                           userInfo: [NSLocalizedDescriptionKey: "model still nil after load"]))
        }
        return model
    }

    // MARK: - Backend self-test

    /// Runs the model on a SYNTHETIC input that any machine can reproduce
    /// bit-exactly, and returns `(inputChecksum, outputChecksum)`.
    ///
    /// The frame-based checksums have a confound: this app decodes the video
    /// itself, so a Mac and a phone can legitimately hand the model different
    /// pixels for "the same frame" and a mismatch would not localise. This has
    /// no decode, no Vision, no warp and no camera intrinsics — just the tensors
    /// and the model.
    ///
    /// Every value is an exact multiple of 1/256 in [-0.5, 0.5), which is
    /// representable in Float16 without rounding, so the input checksum is a
    /// property of the formula rather than of anyone's arithmetic. If two
    /// machines report the same input checksum and different output checksums,
    /// the two Core ML backends compute different things from identical bytes,
    /// with nothing else left in the chain to blame.
    static func backendSelfTest() async throws -> (input: UInt64, output: UInt64) {
        let e = SAM3DPoseEstimator()
        try await e.loadModelIfNeeded()
        let model = try await e.ensureModelLoaded()

        let H = PreprocessingConstants.inputHeight
        let W = PreprocessingConstants.inputWidth
        func synth(_ c: Int, _ y: Int, _ x: Int, _ salt: Int) -> Float16 {
            let n = (x &* 7 &+ y &* 13 &+ c &* 29 &+ salt) & 255
            return Float16(Float(n) / 256.0 - 0.5)
        }

        let image = try makeInputArray(shape: [1, 3, H, W], dataType: .float16)
        let ip = image.dataPointer.bindMemory(to: Float16.self, capacity: image.count)
        var i = 0
        for c in 0..<3 { for y in 0..<H { for x in 0..<W { ip[i] = synth(c, y, x, 0); i += 1 } } }

        let ray = try makeInputArray(shape: [1, 2, H, W], dataType: .float16)
        let rp = ray.dataPointer.bindMemory(to: Float16.self, capacity: ray.count)
        i = 0
        for c in 0..<2 { for y in 0..<H { for x in 0..<W { rp[i] = synth(c, y, x, 91); i += 1 } } }

        let cliff = try makeInputArray(shape: [1, 3], dataType: .float16)
        let cp = cliff.dataPointer.bindMemory(to: Float16.self, capacity: 3)
        cp[0] = Float16(0.03125); cp[1] = Float16(-0.0625); cp[2] = Float16(0.75)

        var inputVals: [Float] = []
        inputVals.reserveCapacity(image.count + ray.count + 3)
        for k in 0..<image.count { inputVals.append(Float(ip[k])) }
        for k in 0..<ray.count { inputVals.append(Float(rp[k])) }
        for k in 0..<3 { inputVals.append(Float(cp[k])) }

        let provider = try MLDictionaryFeatureProvider(dictionary: [
            "image": MLFeatureValue(multiArray: image),
            "ray_map": MLFeatureValue(multiArray: ray),
            "cliff": MLFeatureValue(multiArray: cliff),
        ])
        let out = try await runPrediction(model: model, input: provider)
        guard let jc = out.featureValue(for: "joint_coords")?.multiArrayValue,
              let ct = out.featureValue(for: "cam_t")?.multiArrayValue else {
            throw EstimatorError.missingOutputFeature("joint_coords/cam_t")
        }
        var flat: [Float] = []
        flat.reserveCapacity(jc.count + ct.count)
        for k in 0..<jc.count { flat.append(jc[k].floatValue) }
        for k in 0..<ct.count { flat.append(ct[k].floatValue) }
        return (checksum(inputVals), checksum(flat))
    }

    private static func runPrediction(model: MLModel, input: MLFeatureProvider) async throws -> MLFeatureProvider {
        try await withCheckedThrowingContinuation { (continuation: CheckedContinuation<MLFeatureProvider, Error>) in
            workQueue.async {
                do {
                    let result = try model.prediction(from: input)
                    continuation.resume(returning: result)
                } catch {
                    continuation.resume(throwing: EstimatorError.predictionFailed(error))
                }
            }
        }
    }

    // MARK: - Person bbox (Vision)

    /// Detects a person bbox via `VNDetectHumanRectanglesRequest`; falls back to
    /// the whole image if none is found or detection fails outright — matching
    /// the Python path's own behavior with no detector configured
    /// (`sam_3d_body_estimator.py:125`: `boxes = [0, 0, width, height]`).
    /// Returns the bbox in UPRIGHT pixel coordinates (top-left origin), i.e. the
    /// same space as `uiImage.size` (which already accounts for EXIF orientation).
    /// `upperBodyOnly` MUST be set false, and it is not the default.
    ///
    /// A torso-only box crops the legs out of the square the model sees. Measured
    /// on a 576x768 running clip: the default box spans 20-26% of the image
    /// height and ends above the hips, versus 46-60% reaching the feet. SAM 3D
    /// Body then has no leg pixels to read and emits a near-standing mean pose
    /// for the legs while the torso still tracks — which is what "the skeleton
    /// doesn't match" looked like. Scored against Vision's own 2-D body pose over
    /// 20 frames, leg error was 9.0% of subject height with the default box and
    /// 4.6% with the full-body box, while torso error was unchanged (2.0% vs
    /// 1.9%). Harness: `labs/sam-3d-body/export/box_ablation.py`.
    ///
    /// Exposed as a factory so a test can assert the flag without a Vision run.
    static func makePersonRectangleRequest() -> VNDetectHumanRectanglesRequest {
        let request = VNDetectHumanRectanglesRequest()
        request.upperBodyOnly = false
        return request
    }

    /// The ONE copy of the flip + clamp + degeneracy guard.
    ///
    /// Vision's normalized `boundingBox` has its origin at bottom-left with Y
    /// increasing upward, REGARDLESS of the image's own orientation (Vision
    /// normalizes against the orientation we passed in). This converts it to the
    /// top-left-origin / Y-down pixel space that the rest of this file (and the
    /// Python bbox math it mirrors) uses throughout, intersects it with the
    /// image bounds, and returns `nil` for a degenerate result.
    ///
    /// Extracted 2026-08-14 so `detectPersonBBox` and the external-box seam call
    /// literally the same arithmetic. A second copy of this flip is exactly the
    /// class of defect `PersonBoxTests` exists for: a wrong-signed flip on
    /// `0.303,0.238,0.334,0.414` against 576x1024 reads `y1 = 243.712` instead
    /// of `356.352`, i.e. 112.6 px away, and nothing downstream fails loudly.
    ///
    /// SCOPE, stated precisely: this reproduces ONLY the degenerate-rect branch.
    /// It CANNOT reproduce the handler-throw or zero-observation branches,
    /// because its input is an already-obtained box. A caller that assumes this
    /// helper also encodes detection failure would drop its own not-found check
    /// and reintroduce a silent whole-image path.
    static func personBoxPixels(visionNormalizedBottomLeftOriginYUp nb: CGRect,
                                imageSize: CGSize) -> CGRect? {
        let w = imageSize.width, h = imageSize.height
        let x1 = nb.minX * w
        let y1 = (1 - nb.minY - nb.height) * h
        let rect = CGRect(x: x1, y: y1, width: nb.width * w, height: nb.height * h)
            .intersection(CGRect(origin: .zero, size: imageSize))
        guard rect.width > 1, rect.height > 1 else { return nil }
        return rect
    }

    /// Resolves the person box `estimate(uiImage:)` crops from.
    ///
    /// `injectedNormalizedBottomLeft == nil` is production: run Vision. Non-nil
    /// takes the box VERBATIM through the same `personBoxPixels` conversion and
    /// returns BEFORE any `VNImageRequestHandler` is constructed — which is the
    /// structural reason the injected path cannot silently fall back to the
    /// whole image. It takes a NORMALIZED rect, never a pixel rect, so a caller
    /// cannot hand in an already-converted box and bypass the clamp.
    static func resolvePersonBox(uiImage: UIImage,
                                 injectedNormalizedBottomLeft: CGRect?) throws
        -> (CGRect, usedFallback: Bool) {
        if let injected = injectedNormalizedBottomLeft {
            guard let rect = personBoxPixels(visionNormalizedBottomLeftOriginYUp: injected,
                                             imageSize: uiImage.size) else {
                throw EstimatorError.preprocessingFailed(
                    "the supplied person box is degenerate after the flip and bounds "
                    + "intersection; a caller supplying a box must handle that itself "
                    + "rather than have the whole image substituted silently")
            }
            return (rect, false)
        }
        return try detectPersonBBox(uiImage: uiImage)
    }

    private static func detectPersonBBox(uiImage: UIImage) throws -> (CGRect, usedFallback: Bool) {
        let fallback = (CGRect(origin: .zero, size: uiImage.size), true)
        guard let cgImage = uiImage.cgImage else { throw EstimatorError.imageDecodeFailed }

        let request = makePersonRectangleRequest()
        let handler = VNImageRequestHandler(cgImage: cgImage,
                                             orientation: cgOrientation(for: uiImage.imageOrientation),
                                             options: [:])
        do {
            try handler.perform([request])
        } catch {
            return fallback
        }

        let observations = (request.results as? [VNHumanObservation]) ?? []
        guard let best = observations.max(by: { $0.confidence < $1.confidence }) else {
            return fallback
        }

        guard let rect = personBoxPixels(visionNormalizedBottomLeftOriginYUp: best.boundingBox,
                                         imageSize: uiImage.size) else { return fallback }
        return (rect, false)
    }

    private static func cgOrientation(for orientation: UIImage.Orientation) -> CGImagePropertyOrientation {
        switch orientation {
        case .up: return .up
        case .upMirrored: return .upMirrored
        case .down: return .down
        case .downMirrored: return .downMirrored
        case .left: return .left
        case .leftMirrored: return .leftMirrored
        case .right: return .right
        case .rightMirrored: return .rightMirrored
        @unknown default: return .up
        }
    }

    // MARK: - Crop geometry (bbox -> affine warp params)

    /// Everything downstream (image warp, ray_map, cliff) is a pure function of
    /// this struct. `bboxCenter`/`side` are in ORIGINAL (upright) image pixels.
    ///
    /// Derivation (verified against the real `GetBBoxCenterScale` +
    /// `TopdownAffine(input_size=(512,512))` pipeline, see file header):
    ///   bw    = bbox_w * 1.25                 -- GetBBoxCenterScale pads w,h
    ///   bh    = bbox_h * 1.25                    independently by 1.25
    ///   side  = max(bw / 0.75, bh)
    ///
    /// `side` is NOT `max(bw, bh)`. TopdownAffine calls `fix_aspect_ratio`
    /// twice: first to the 3:4 prior (`bbox_utils.py`, aspect 0.75), then to the
    /// model's 1:1 input. The pair collapses to `max(bw / 0.75, bh)`, which is
    /// wider than the naive square whenever the padded box is wider than 3:4.
    /// Getting this wrong shifts the crop: `export/CONTRACT.md` measures a
    /// `cliff[2]` error of 0.24 and joint errors of tens of millimetres.
    ///   scale = 512 / side
    ///   tx    = 256 - bboxCenter.x * scale
    ///   ty    = 256 - bboxCenter.y * scale
    /// This is exactly `get_warp_matrix(center, (side,side), rot=0, (512,512))`
    /// for the rot=0 case (cv2.getAffineTransform-verified numerically equal).
    struct CropGeometry {
        let bboxCenter: CGPoint
        let side: CGFloat

        var scale: CGFloat { CGFloat(PreprocessingConstants.squareWarpSize) / side }
        var tx: CGFloat { CGFloat(PreprocessingConstants.squareWarpSize) / 2 - bboxCenter.x * scale }
        var ty: CGFloat { CGFloat(PreprocessingConstants.squareWarpSize) / 2 - bboxCenter.y * scale }
    }

    private static func computeCropGeometry(bbox: CGRect, imageSize: CGSize) -> CropGeometry {
        let center = CGPoint(x: bbox.midX, y: bbox.midY)
        let paddedW = bbox.width * PreprocessingConstants.bboxPadding
        let paddedH = bbox.height * PreprocessingConstants.bboxPadding
        let side = max(max(paddedW / PreprocessingConstants.bboxPriorAspect, paddedH), 1)
        return CropGeometry(bboxCenter: center, side: side)
    }

    // MARK: - Image warp + tensor fill

    /// Renders the ORIGINAL (upright, orientation-corrected) image through the
    /// forward affine `dst = src*scale + t` into a 512x512 RGBA8 buffer, top-left
    /// origin / row-major, matching `cv2.warpAffine(img, warp_mat, (512,512),
    /// flags=cv2.INTER_LINEAR)`'s output layout.
    ///
    /// Uses `UIGraphicsImageRenderer` (not a raw `CGContext`) specifically because
    /// its context — and `UIImage.draw(in:)` — are both documented to operate in
    /// UIKit's top-left-origin/Y-down coordinate space, which matches the
    /// Python-derived (scale,tx,ty) formula above exactly with no manual Y-flip.
    /// A raw `CGContext(data:...)` defaults to bottom-left/Y-up and would silently
    /// invert the result if this transform were concatenated onto it directly.
    ///
    /// RESIDUAL RISK (flagged, not fixed here — see README): `interpolationQuality
    /// = .high` is CoreGraphics' best available resampling but is not documented
    /// to be exactly bilinear; cv2's `INTER_LINEAR` is strictly bilinear. This
    /// codebase has already hit a confirmed instance of ".high" not being
    /// equivalent to a Python-side kernel (see wiki: AutoLevel's S336
    /// Pillow/Lanczos-vs-CoreGraphics finding) — worth an empirical check once the
    /// model is running, not fixable without a device to measure on.
    private static func renderWarpedRGBA(source: UIImage, geometry: CropGeometry) -> [UInt8]? {
        let size = PreprocessingConstants.squareWarpSize
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1.0
        format.opaque = true
        let renderer = UIGraphicsImageRenderer(size: CGSize(width: size, height: size), format: format)
        let warped = renderer.image { rendererContext in
            let cg = rendererContext.cgContext
            cg.interpolationQuality = .high
            cg.concatenate(CGAffineTransform(a: geometry.scale, b: 0, c: 0, d: geometry.scale,
                                              tx: geometry.tx, ty: geometry.ty))
            source.draw(in: CGRect(x: 0, y: 0, width: source.size.width, height: source.size.height))
        }
        guard let cgImage = warped.cgImage else { return nil }

        var buffer = [UInt8](repeating: 0, count: size * size * 4)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let ctx = CGContext(data: &buffer, width: size, height: size, bitsPerComponent: 8,
                                   bytesPerRow: size * 4, space: colorSpace,
                                   bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue) else { return nil }
        // A plain identity-rect `draw(in:)` here — no geometric transform is
        // concatenated, so the Y-up-vs-Y-down subtlety above does not apply; this
        // is the standard, ubiquitous "get upright RGBA8 bytes from a CGImage"
        // idiom.
        ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: size, height: size))
        return buffer
    }

    private static func makeInputArray(shape: [Int], dataType: MLMultiArrayDataType) throws -> MLMultiArray {
        try MLMultiArray(shape: shape.map { NSNumber(value: $0) }, dataType: dataType)
    }

    private static func makeImageTensor(rgba: [UInt8]) throws -> MLMultiArray {
        let H = PreprocessingConstants.inputHeight
        let W = PreprocessingConstants.inputWidth
        let squareSize = PreprocessingConstants.squareWarpSize
        let margin = PreprocessingConstants.widthCropMargin
        let mean = PreprocessingConstants.imageMean
        let std = PreprocessingConstants.imageStd

        let array = try makeInputArray(shape: [1, 3, H, W], dataType: .float16)
        let ptr = array.dataPointer.bindMemory(to: Float16.self, capacity: 3 * H * W)

        rgba.withUnsafeBufferPointer { buf in
            for h in 0..<H {
                let srcRowBase = h * squareSize * 4
                for wOut in 0..<W {
                    let srcCol = wOut + margin
                    let srcIdx = srcRowBase + srcCol * 4
                    let r = Float(buf[srcIdx + 0]) / 255.0
                    let g = Float(buf[srcIdx + 1]) / 255.0
                    let b = Float(buf[srcIdx + 2]) / 255.0
                    let outBase = h * W + wOut
                    ptr[0 * H * W + outBase] = Float16((r - mean[0]) / std[0])
                    ptr[1 * H * W + outBase] = Float16((g - mean[1]) / std[1])
                    ptr[2 * H * W + outBase] = Float16((b - mean[2]) / std[2])
                }
            }
        }
        return array
    }

    /// Per-pixel camera ray field. Closed form derived from, and numerically
    /// verified against, `get_ray_condition` in `sam3d_body.py` (called at its
    /// real call-time shape: full 512x512 square, THEN the SAME `[...,64:-64]`
    /// width crop the image tensor gets). For output pixel (h, wOut):
    ///   origX = bboxCenter.x + (wOut + 64 - 256) * (side/512)
    ///   origY = bboxCenter.y + (h        - 256) * (side/512)
    ///   ray0  = (origX - camIntCx) / f
    ///   ray1  = (origY - camIntCy) / f
    /// Verified to match the library function exactly (< 1e-9) at both corners,
    /// the center, and two off-center points of a synthetic 800x600/portrait-bbox
    /// example.
    private static func makeRayMapTensor(geometry: CropGeometry, focalLength: Float,
                                          camIntCx: Float, camIntCy: Float) throws -> MLMultiArray {
        let H = PreprocessingConstants.inputHeight
        let W = PreprocessingConstants.inputWidth
        let margin = Float(PreprocessingConstants.widthCropMargin)
        let squareSize = Float(PreprocessingConstants.squareWarpSize)
        let side = Float(geometry.side)
        let cx = Float(geometry.bboxCenter.x)
        let cy = Float(geometry.bboxCenter.y)
        let pixelToOrig = side / squareSize

        let array = try makeInputArray(shape: [1, 2, H, W], dataType: .float16)
        let ptr = array.dataPointer.bindMemory(to: Float16.self, capacity: 2 * H * W)

        for h in 0..<H {
            let origY = cy + (Float(h) - squareSize / 2) * pixelToOrig
            let ray1 = (origY - camIntCy) / focalLength
            for wOut in 0..<W {
                let origX = cx + (Float(wOut) + margin - squareSize / 2) * pixelToOrig
                let ray0 = (origX - camIntCx) / focalLength
                let base = h * W + wOut
                ptr[0 * H * W + base] = Float16(ray0)
                ptr[1 * H * W + base] = Float16(ray1)
            }
        }
        return array
    }

    /// CLIFF condition `[(cx-cx_int)/f, (cy-cy_int)/f, side/f]`. Verified equal to
    /// `_get_decoder_condition`'s `cliff` branch given the default `cam_int` (see
    /// `makeRayMapTensor`'s doc — same verification run confirmed `USE_INTRIN_CENTER`
    /// true/false are numerically identical here).
    private static func makeCliffTensor(geometry: CropGeometry, focalLength: Float,
                                         camIntCx: Float, camIntCy: Float) throws -> MLMultiArray {
        let array = try makeInputArray(shape: [1, 3], dataType: .float16)
        let ptr = array.dataPointer.bindMemory(to: Float16.self, capacity: 3)
        ptr[0] = Float16((Float(geometry.bboxCenter.x) - camIntCx) / focalLength)
        ptr[1] = Float16((Float(geometry.bboxCenter.y) - camIntCy) / focalLength)
        ptr[2] = Float16(Float(geometry.side) / focalLength)
        return array
    }

    // MARK: - Output parsing

    static func parseOutput(_ provider: MLFeatureProvider, usedFallbackBBox: Bool,
                            inputChecksum: UInt64,
                            sourceHash: UInt64, bboxHash: UInt64,
                            warpHash: UInt64) throws -> Output {
        guard let jointCoordsArray = provider.featureValue(for: "joint_coords")?.multiArrayValue else {
            throw EstimatorError.missingOutputFeature("joint_coords")
        }
        guard let globalRotsArray = provider.featureValue(for: "global_rots")?.multiArrayValue else {
            throw EstimatorError.missingOutputFeature("global_rots")
        }
        guard let camTArray = provider.featureValue(for: "cam_t")?.multiArrayValue else {
            throw EstimatorError.missingOutputFeature("cam_t")
        }
        guard let keypoints2DArray = provider.featureValue(for: "keypoints_2d")?.multiArrayValue else {
            throw EstimatorError.missingOutputFeature("keypoints_2d")
        }

        for (name, array) in [
            ("joint_coords", jointCoordsArray),
            ("global_rots", globalRotsArray),
            ("cam_t", camTArray),
            ("keypoints_2d", keypoints2DArray),
        ] where array.dataType != .float32 {
            throw EstimatorError.unexpectedOutputDataType(
                "\(name): expected Float32, got \(outputDataTypeName(array.dataType))"
            )
        }

        let jointCoords = try readVec3Array(jointCoordsArray, count: PreprocessingConstants.numBodyJoints, name: "joint_coords")
        guard jointCoords.allSatisfy(MHRRetarget.isValidSourceJointCoordinate) else {
            throw EstimatorError.invalidOutputValue(
                "joint_coords must remain inside the supported metric coordinate domain"
            )
        }
        let globalRots = try readMat3x3Array(globalRotsArray, count: PreprocessingConstants.numBodyJoints, name: "global_rots")
        let camT = try readVec3(camTArray, name: "cam_t")
        guard MHRRetarget.isValidCameraTranslation(camT) else {
            throw EstimatorError.invalidOutputValue(
                "cam_t must contain bounded finite x/y values and bounded positive depth"
            )
        }
        let keypoints2D = try readVec2Array(keypoints2DArray, count: PreprocessingConstants.numOutputKeypoints2D, name: "keypoints_2d")

        // Checksum the joints exactly as the model returned them, before any
        // retarget or projection, so a mismatch localises to inference itself.
        var flat: [Float] = []
        flat.reserveCapacity(jointCoords.count * 3)
        for j in jointCoords { flat.append(j.x); flat.append(j.y); flat.append(j.z) }
        flat.append(camT.x); flat.append(camT.y); flat.append(camT.z)

        return Output(jointCoords: jointCoords, globalRots: globalRots, camT: camT,
                      keypoints2D: keypoints2D, usedFallbackBBox: usedFallbackBBox,
                      inputChecksum: inputChecksum,
                      outputChecksum: Self.checksum(flat),
                      sourceHash: sourceHash, bboxHash: bboxHash, warpHash: warpHash)
    }

    @inline(__always)
    private static func idx(_ values: Int...) -> [NSNumber] { values.map { NSNumber(value: $0) } }

    private static func outputDataTypeName(_ dataType: MLMultiArrayDataType) -> String {
        if dataType == .float16 { return "Float16" }
        if dataType == .float32 { return "Float32" }
        if dataType == .double { return "Double" }
        if dataType == .int32 { return "Int32" }
        return "raw type \(dataType.rawValue)"
    }

    private static func readVec3Array(_ array: MLMultiArray, count: Int, name: String) throws -> [SIMD3<Float>] {
        guard array.shape.map(\.intValue) == [count, 3] else {
            throw EstimatorError.unexpectedOutputShape("\(name): expected [\(count),3], got \(array.shape)")
        }
        var result: [SIMD3<Float>] = []
        result.reserveCapacity(count)
        for i in 0..<count {
            let x = array[idx(i, 0)].floatValue
            let y = array[idx(i, 1)].floatValue
            let z = array[idx(i, 2)].floatValue
            guard x.isFinite, y.isFinite, z.isFinite else {
                throw EstimatorError.invalidOutputValue(
                    "\(name)[\(i)] must contain three finite coordinates"
                )
            }
            result.append(SIMD3<Float>(x, y, z))
        }
        return result
    }

    private static func readVec2Array(_ array: MLMultiArray, count: Int, name: String) throws -> [SIMD2<Float>] {
        guard array.shape.map(\.intValue) == [count, 2] else {
            throw EstimatorError.unexpectedOutputShape("\(name): expected [\(count),2], got \(array.shape)")
        }
        var result: [SIMD2<Float>] = []
        result.reserveCapacity(count)
        for i in 0..<count {
            let x = array[idx(i, 0)].floatValue
            let y = array[idx(i, 1)].floatValue
            guard x.isFinite, y.isFinite else {
                throw EstimatorError.invalidOutputValue(
                    "\(name)[\(i)] must contain two finite coordinates"
                )
            }
            result.append(SIMD2<Float>(x, y))
        }
        return result
    }

    private static func readVec3(_ array: MLMultiArray, name: String) throws -> SIMD3<Float> {
        guard array.shape.map(\.intValue) == [3] else {
            throw EstimatorError.unexpectedOutputShape(
                "\(name): expected [3], got \(array.shape)"
            )
        }
        return SIMD3<Float>(array[idx(0)].floatValue, array[idx(1)].floatValue, array[idx(2)].floatValue)
    }

    /// PyTorch stores each joint's rotation row-major (`R[row][col]`); `simd_float3x3`'s
    /// memberwise initializer takes COLUMN vectors, so column `c`'s `r`-th component
    /// must be `R[r][c]`. This makes `simdMatrix * simd_vector` equal PyTorch's `R @ v`.
    private static func readMat3x3Array(_ array: MLMultiArray, count: Int, name: String) throws -> [simd_float3x3] {
        guard array.shape.map(\.intValue) == [count, 3, 3] else {
            throw EstimatorError.unexpectedOutputShape("\(name): expected [\(count),3,3], got \(array.shape)")
        }
        var result: [simd_float3x3] = []
        result.reserveCapacity(count)
        for i in 0..<count {
            func finiteValue(_ row: Int, _ column: Int) throws -> Float {
                let value = array[idx(i, row, column)].floatValue
                guard value.isFinite else {
                    throw EstimatorError.invalidOutputValue(
                        "\(name)[\(i)][\(row)][\(column)] must be finite"
                    )
                }
                return value
            }
            let col0 = try SIMD3<Float>(finiteValue(0, 0), finiteValue(1, 0), finiteValue(2, 0))
            let col1 = try SIMD3<Float>(finiteValue(0, 1), finiteValue(1, 1), finiteValue(2, 1))
            let col2 = try SIMD3<Float>(finiteValue(0, 2), finiteValue(1, 2), finiteValue(2, 2))
            result.append(simd_float3x3(col0, col1, col2))
        }
        return result
    }
}
