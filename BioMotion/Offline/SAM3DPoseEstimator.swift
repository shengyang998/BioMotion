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
///   cliff   : [1,3] Float16, [(cx-cx_int)/f, (cy-cy_int)/f, bbox_w/f].
///   joint_coords/global_rots/cam_t/keypoints_2d outputs, Float32, per contract.
///
/// `CONTRACT.md` (labs/sam-3d-body/export/CONTRACT.md) did not exist yet when this
/// file was written. Every geometric formula in `PreprocessingConstants` and the
/// private helpers below was instead derived directly from the released Python
/// source and cross-checked empirically by running the actual PyTorch transform
/// functions (`GetBBoxCenterScale`, `TopdownAffine`, `get_ray_condition`,
/// `prepare_batch`'s default `cam_int`) against this Swift port's closed-form
/// formulas on concrete numeric examples — every value matched exactly. See the
/// citations inline and `BioMotion/Offline/README.md` for the full derivation and
/// residual risk. CROSS-CHECK AGAINST CONTRACT.md ONCE IT EXISTS regardless.
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
        /// Camera translation, model units.
        let camT: SIMD3<Float>
        /// 70 2D keypoints in the 512x384 crop pixel frame (x, y).
        let keypoints2D: [SIMD2<Float>]
        /// True if Vision found no person and preprocessing fell back to the
        /// whole image as the "bbox" — matches the Python path's own fallback
        /// (`sam_3d_body_estimator.py:125`, `boxes = [0,0,width,height]` when no
        /// detector is configured). Informational only; the estimate still runs.
        let usedFallbackBBox: Bool

        /// Checksum of the normalised `image` tensor actually handed to Core ML,
        /// and of the returned `joint_coords`.
        ///
        /// These exist to settle one specific question. On a real frame the Mac
        /// and the phone disagree about the pose — the Mac places a sprinter's
        /// recovery foot raised behind, the phone places it on the ground —
        /// while the Mac's own prediction is invariant to person-box changes,
        /// ±2 LSB pixel noise and RGB/BGR swaps. So the divergence is not in the
        /// input, unless the input differs in a way none of those probes model.
        ///
        /// Matching input checksums with differing output checksums proves the
        /// two Core ML backends compute different things from identical bytes.
        /// Differing input checksums instead point at preprocessing, and the
        /// difference is then reproducible on this machine.
        ///
        /// Both are order-sensitive FNV-1a over the raw bit patterns, so a
        /// single flipped mantissa bit changes them.
        let inputChecksum: UInt64
        let outputChecksum: UInt64
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

    enum EstimatorError: LocalizedError {
        case imageDecodeFailed
        case modelLoadFailed(Error)
        case preprocessingFailed(String)
        case predictionFailed(Error)
        case missingOutputFeature(String)
        case unexpectedOutputShape(String)

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
                return "Pose model output is missing expected field \"\(name)\" — check SAM3DBodyPose.mlpackage matches the frozen contract."
            case .unexpectedOutputShape(let detail):
                return "Pose model output shape mismatch: \(detail)"
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
    /// weights. Exposed so a view can show live progress; the message on the
    /// error thrown by `loadModelIfNeeded()` already carries the same numbers.
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
    /// when the pack has not arrived yet, throws promptly with a message that
    /// states the download percentage — this call never blocks on the transfer.
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

    func estimate(uiImage: UIImage) async throws -> Output {
        let model = try await ensureModelLoaded()

        guard uiImage.size.width > 0, uiImage.size.height > 0 else {
            throw EstimatorError.imageDecodeFailed
        }

        let (bboxRect, usedFallback) = try Self.detectPersonBBox(uiImage: uiImage)
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
                                    inputChecksum: inputChecksum)
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
    private static func detectPersonBBox(uiImage: UIImage) throws -> (CGRect, usedFallback: Bool) {
        let fallback = (CGRect(origin: .zero, size: uiImage.size), true)
        guard let cgImage = uiImage.cgImage else { throw EstimatorError.imageDecodeFailed }

        let request = VNDetectHumanRectanglesRequest()
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

        // Vision's normalized boundingBox has its origin at bottom-left with Y
        // increasing upward, REGARDLESS of the image's own orientation (Vision
        // normalizes against the orientation we passed in). Convert to the
        // top-left-origin / Y-down pixel space that the rest of this file (and
        // the Python bbox math it mirrors) uses throughout.
        let w = uiImage.size.width, h = uiImage.size.height
        let nb = best.boundingBox
        let x1 = nb.minX * w
        let y1 = (1 - nb.minY - nb.height) * h
        let rect = CGRect(x: x1, y: y1, width: nb.width * w, height: nb.height * h)
            .intersection(CGRect(origin: .zero, size: uiImage.size))
        guard rect.width > 1, rect.height > 1 else { return fallback }
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

    private static func parseOutput(_ provider: MLFeatureProvider, usedFallbackBBox: Bool,
                                    inputChecksum: UInt64) throws -> Output {
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

        let jointCoords = try readVec3Array(jointCoordsArray, count: PreprocessingConstants.numBodyJoints, name: "joint_coords")
        let globalRots = try readMat3x3Array(globalRotsArray, count: PreprocessingConstants.numBodyJoints, name: "global_rots")
        let camT = try readVec3(camTArray, name: "cam_t")
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
                      outputChecksum: Self.checksum(flat))
    }

    @inline(__always)
    private static func idx(_ values: Int...) -> [NSNumber] { values.map { NSNumber(value: $0) } }

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
            result.append(SIMD2<Float>(array[idx(i, 0)].floatValue, array[idx(i, 1)].floatValue))
        }
        return result
    }

    private static func readVec3(_ array: MLMultiArray, name: String) throws -> SIMD3<Float> {
        guard array.count == 3 else {
            throw EstimatorError.unexpectedOutputShape("\(name): expected 3 elements, got \(array.count)")
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
            func v(_ r: Int, _ c: Int) -> Float { array[idx(i, r, c)].floatValue }
            let col0 = SIMD3<Float>(v(0, 0), v(1, 0), v(2, 0))
            let col1 = SIMD3<Float>(v(0, 1), v(1, 1), v(2, 1))
            let col2 = SIMD3<Float>(v(0, 2), v(1, 2), v(2, 2))
            result.append(simd_float3x3(col0, col1, col2))
        }
        return result
    }
}
