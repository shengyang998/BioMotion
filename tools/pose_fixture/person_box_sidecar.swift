// macOS HOST tool. Computes the person box per SAMPLED frame of one clip and
// writes ONE sidecar JSON the iOS-Simulator fixture generator CONSUMES instead
// of calling `SAM3DPoseEstimator.detectPersonBBox` live.
//
// WHY THIS EXISTS (measured 2026-08-14, /tmp/simvisionprobe/run.log).
// `VNDetectHumanRectanglesRequest.perform` THREW on 12/12 sampled frames inside
// the iOS Simulator -- `Error Domain=com.apple.Vision Code=9 "Could not create
// inference context"` -- with the face-rect and body-pose controls throwing
// their own distinct messages, while decode in that same host is proven healthy
// (`decode_failed=0`, `decoded=576x1024 bpp=32`, `lum_mean` 102.75-110.11).
// macOS Vision, same request configuration, found a person on 12/12 sampled
// frames of BOTH clips (/tmp/visionprobe/run.log). The Simulator has no Vision
// ML inference backend; that is an environment limitation, not a defect in the
// generator or its plumbing.
//
// PROVENANCE HONESTY. macOS Vision provenance is NOT iOS Vision provenance.
// Every box this tool emits is recorded as `bbox_source macos_vision INTERIM`
// downstream. That is a recorded fact, never "production-identical".
//
// WHY IT CANNOT LINK THE APP'S SOURCES. `FrameSource.swift` and
// `SAM3DPoseEstimator.swift` both `import UIKit`, and `project.yml` declares
// only `platform: iOS` targets, so there is no macOS target to link against.
// The sampling arithmetic and the decode conventions are therefore
// REIMPLEMENTED here with the constants inlined. That duplication is NOT
// trusted: the generator recomputes the plan through the real
// `FrameSource.sampleTimestamps` and refuses on ONE ULP of disagreement
// (the timestamp gate, registration section 6 part B).
//
// Usage:
//   person_box_sidecar --clip <video_012|...> --out <dir> --repo-root <path> <video>
//
// The output DIRECTORY is operator-chosen; the FILENAME is derived
// (`person_box_sidecar_<clip>.json`) and the tool REFUSES an output directory
// that resolves -- after symlink resolution -- under `--repo-root`. The sidecar
// is a derivative of personal footage and never enters the repository.
//
// No personal absolute path appears in this file: every path arrives as an
// argument, and the tool fails closed with a clear message when one is absent.

import Foundation
import AVFoundation
import CoreGraphics
import CryptoKit
import ImageIO
import Vision

// MARK: - Registered constants, inlined from FrameSource.swift

/// `FrameSource.analysisWindowSeconds` (FrameSource.swift:75).
let kAnalysisWindowSeconds: Double = 8.0
/// `FrameSource.minimumAnalysisSeconds` (:109).
let kMinimumAnalysisSeconds: Double = 2.5
/// `FrameSource.plausibleFrameRates` (:136).
let kPlausibleFrameRates: ClosedRange<Double> = 1.0...240.0
/// `FrameSource.assumedFrameRateWhenUnknown` (:132).
let kAssumedFrameRateWhenUnknown: Double = 30.0
/// `FrameSource.maxNativeWindowFrames` = Int(2.5 * 240.0) + 1 (:104).
let kMaxNativeWindowFrames = Int(kMinimumAnalysisSeconds * kPlausibleFrameRates.upperBound) + 1
/// `FrameSource.maxFramesPerRun` (:70).
let kMaxFramesPerRun = 120
/// `FrameSource.bytesPerDecodedPixel` (:144).
let kBytesPerDecodedPixel = 4
/// `FrameSource.decodedRowAlignmentBytes` (:152).
let kDecodedRowAlignmentBytes = 64
/// `FrameSource.decodedWindowBudgetBytes` (:213-214).
let kDecodedWindowBudgetBytes = kMaxFramesPerRun * 1920 * 1080 * kBytesPerDecodedPixel
/// `FrameSource.VideoDecoder.decodeFrame`'s `CMTime(preferredTimescale:)` (:471).
let kPreferredTimescale: Int32 = 600

let kSchema = "biomotion.person_box_sidecar.v1"
let kBBoxSpace = "vision_normalized_bottom_left_origin_y_up"
let kSourceHashAlgorithm =
    "fnv1a64_over_premultipliedLast_deviceRGB_8bpc_rowBytes_eq_w_times_4"
/// The bare FNV-1a offset basis. Rejected as a NO-PIXELS sentinel on BOTH sides,
/// closing `SAM3DPoseEstimator.sourcePixels`' empty-array degenerate case
/// (SAM3DPoseEstimator.swift:110-111 returns `[]` when `cgImage` is nil).
let kFNVOffsetBasis: UInt64 = 0xcbf29ce484222325

// MARK: - Field-class sanitization (registration section 4)

/// VERSION / RECEIPT class: newlines become SPACES; `/` is PERMITTED, because a
/// version string legitimately contains one (`xcodebuild -version` and
/// `swiftc --version` both do).
func sanitizedReceipt(_ raw: String) -> String {
    raw.replacingOccurrences(of: "\r\n", with: " ")
        .replacingOccurrences(of: "\n", with: " ")
        .replacingOccurrences(of: "\r", with: " ")
        .trimmingCharacters(in: .whitespacesAndNewlines)
}

// MARK: - Sampling plan, reimplemented from FrameSource.sampleTimestamps

/// `FrameSource.sanitisedFrameRate` (:376-381). Neither existing prototype
/// applied this clamp; a clip with a 0 or non-finite nominal rate would
/// otherwise diverge from the generator's plan silently.
func sanitisedFrameRate(_ raw: Double) -> Double {
    guard raw.isFinite, kPlausibleFrameRates.contains(raw) else {
        return kAssumedFrameRateWhenUnknown
    }
    return raw
}

struct SamplingPlan {
    let duration: Double
    let fps: Double
    let step: Double
    let start: Double
    let wanted: Int
    let available: Int
    let count: Int
    let timestamps: [Double]
}

/// The `.nativeWindow` branch of `FrameSource.sampleTimestamps` (:347-370),
/// arithmetic for arithmetic.
func makeSamplingPlan(duration: Double, nominalFrameRate: Double) -> SamplingPlan {
    let fps = sanitisedFrameRate(nominalFrameRate)
    let step = 1.0 / fps
    let wanted = min(kMaxNativeWindowFrames, max(1, Int(kAnalysisWindowSeconds / step)))
    let available = max(1, Int(duration / step))
    let count = min(wanted, available)
    let span = Double(count - 1) * step
    let start = max(0, (duration - span) / 2)
    let timestamps = (0..<count).map { start + Double($0) * step }
    return SamplingPlan(duration: duration, fps: fps, step: step, start: start,
                        wanted: wanted, available: available, count: count,
                        timestamps: timestamps)
}

// MARK: - Decode-size cap, reimplemented from FrameSource.maximumDecodedSize

/// `FrameSource.decodedFrameBytes` (:155-161).
func decodedFrameBytes(width: Int, height: Int) -> Int {
    guard width > 0, height > 0 else { return 0 }
    let row = width * kBytesPerDecodedPixel
    let aligned = ((row + kDecodedRowAlignmentBytes - 1) / kDecodedRowAlignmentBytes)
        * kDecodedRowAlignmentBytes
    return aligned * height
}

/// `FrameSource.decodedSize(naturalSize:cappedToSquareSide:)` (:167-172).
func decodedSize(naturalSize: CGSize, cappedToSquareSide side: CGFloat) -> CGSize {
    let w = naturalSize.width, h = naturalSize.height
    guard w > 0, h > 0, side > 0 else { return .zero }
    let scale = min(1.0, side / max(w, h))
    return CGSize(width: (w * scale).rounded(.down), height: (h * scale).rounded(.down))
}

/// `FrameSource.largestSideFittingBudget` (:270-281).
func largestSideFittingBudget(naturalSize: CGSize, from start: Double, frames: Int) -> CGFloat {
    var side = CGFloat(max(1.0, start))
    while side > 1 {
        let out = decodedSize(naturalSize: naturalSize, cappedToSquareSide: side)
        let cost = frames * decodedFrameBytes(width: Int(out.width), height: Int(out.height))
        if cost <= kDecodedWindowBudgetBytes { break }
        side -= 1
    }
    return side
}

/// `FrameSource.worstCaseDecodedFrameCount` (:222-227).
func worstCaseDecodedFrameCount(duration: Double, nominalFrameRate: Double) -> Int {
    let native = makeSamplingPlan(duration: duration,
                                  nominalFrameRate: nominalFrameRate).count
    return max(1, max(native, kMaxFramesPerRun))
}

/// `FrameSource.maximumDecodedSize(naturalSize:duration:nominalFrameRate:)` (:242-267).
func maximumDecodedSize(naturalSize: CGSize, duration: Double,
                        nominalFrameRate: Double) -> CGSize {
    let frames = worstCaseDecodedFrameCount(duration: duration,
                                            nominalFrameRate: nominalFrameRate)
    let pixelBudget = Double(kDecodedWindowBudgetBytes)
        / Double(frames) / Double(kBytesPerDecodedPixel)
    let w = Double(naturalSize.width), h = Double(naturalSize.height)
    guard w.isFinite, h.isFinite, w > 0, h > 0 else {
        let square = pixelBudget.squareRoot().rounded(.down)
        let side = largestSideFittingBudget(
            naturalSize: CGSize(width: square, height: square), from: square, frames: frames)
        return CGSize(width: side, height: side)
    }
    let scale = min(1.0, (pixelBudget / (w * h)).squareRoot())
    let side = largestSideFittingBudget(naturalSize: naturalSize,
                                        from: (max(w, h) * scale).rounded(.down),
                                        frames: frames)
    return CGSize(width: side, height: side)
}

// MARK: - Source-pixel hash + luminance receipts

/// Byte-for-byte the algorithm of `SAM3DPoseEstimator.checksumBytes`
/// (SAM3DPoseEstimator.swift:102-106) over `sourcePixels` (:110-122), driven
/// from a `CGImage` because macOS has no `UIImage`. Same `CGContext`
/// parameters: 8 bits per component, `bytesPerRow = w * 4`,
/// `CGColorSpaceCreateDeviceRGB()`, `premultipliedLast`.
func sourcePixels(_ cg: CGImage) -> [UInt8] {
    let w = cg.width, h = cg.height
    var buf = [UInt8](repeating: 0, count: w * h * 4)
    buf.withUnsafeMutableBytes { raw in
        guard let ctx = CGContext(data: raw.baseAddress, width: w, height: h,
                                  bitsPerComponent: 8, bytesPerRow: w * 4,
                                  space: CGColorSpaceCreateDeviceRGB(),
                                  bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)
        else { return }
        ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
    }
    return buf
}

func fnv1aBytes(_ bytes: [UInt8]) -> UInt64 {
    var h: UInt64 = kFNVOffsetBasis
    for b in bytes { h = (h ^ UInt64(b)) &* 0x100000001b3 }
    return h
}

/// Rec.709 luma over EVERY pixel of the same deviceRGB buffer the hash is taken
/// from. Receipt only -- it is emitted as a delta when content binding reads
/// UNPROVEN, never as a gate. The generator implements the identical formula.
func luminanceStats(_ buf: [UInt8]) -> (mean: Double, sd: Double) {
    guard buf.count >= 4 else { return (0, 0) }
    var sum = 0.0, sumSq = 0.0
    var n = 0
    var i = 0
    while i + 3 < buf.count {
        let l = 0.2126 * Double(buf[i]) + 0.7152 * Double(buf[i + 1]) + 0.0722 * Double(buf[i + 2])
        sum += l
        sumSq += l * l
        n += 1
        i += 4
    }
    guard n > 0 else { return (0, 0) }
    let mean = sum / Double(n)
    return (mean, max(0, sumSq / Double(n) - mean * mean).squareRoot())
}

// MARK: - JSON emission helpers

/// Exact-integer decimal string of a Double's IEEE-754 bit pattern. Every value
/// that participates in an equality gate travels this way, so the comparison is
/// immune to JSON decimal formatting and to `Double ==` semantics.
func bits(_ v: Double) -> String { String(v.bitPattern) }

func g17(_ v: Double) -> String { String(format: "%.17g", v) }

func jsonString(_ s: String) -> String {
    var out = "\""
    for ch in s.unicodeScalars {
        switch ch {
        case "\"": out += "\\\""
        case "\\": out += "\\\\"
        case "\n": out += "\\n"
        case "\r": out += "\\r"
        case "\t": out += "\\t"
        default:
            if ch.value < 0x20 {
                out += String(format: "\\u%04x", ch.value)
            } else {
                out.unicodeScalars.append(ch)
            }
        }
    }
    return out + "\""
}

func fail(_ message: String) -> Never {
    FileHandle.standardError.write(Data(("PERSON-BOX-SIDECAR-FAIL " + message + "\n").utf8))
    exit(1)
}

func log(_ line: String) {
    print("PERSON-BOX-SIDECAR " + line)
    fflush(stdout)
}

// MARK: - Shell receipts

func runTool(_ path: String, _ args: [String]) -> String {
    let p = Process()
    p.executableURL = URL(fileURLWithPath: path)
    p.arguments = args
    let pipe = Pipe()
    p.standardOutput = pipe
    p.standardError = FileHandle.nullDevice
    do { try p.run() } catch { return "unavailable" }
    let data = pipe.fileHandleForReading.readDataToEndOfFile()
    p.waitUntilExit()
    let text = String(data: data, encoding: .utf8) ?? ""
    let cleaned = sanitizedReceipt(text)
    return cleaned.isEmpty ? "unavailable" : cleaned
}

// MARK: - Argument parsing

var clipId: String?
var outDir: String?
var repoRoot: String?
var videoPath: String?
var argv = Array(CommandLine.arguments.dropFirst())
var ai = 0
while ai < argv.count {
    switch argv[ai] {
    case "--clip":
        ai += 1
        guard ai < argv.count else { fail("--clip needs a value") }
        clipId = argv[ai]
    case "--out":
        ai += 1
        guard ai < argv.count else { fail("--out needs a value") }
        outDir = argv[ai]
    case "--repo-root":
        ai += 1
        guard ai < argv.count else { fail("--repo-root needs a value") }
        repoRoot = argv[ai]
    default:
        guard videoPath == nil else { fail("unexpected extra argument \(argv[ai])") }
        videoPath = argv[ai]
    }
    ai += 1
}

guard let clip = clipId, !clip.isEmpty else {
    fail("--clip <id> is required (it derives the sidecar filename)")
}
guard !clip.contains("/") else { fail("--clip may not contain a path separator") }
guard let out = outDir, !out.isEmpty else { fail("--out <directory> is required") }
// `--repo-root` is REQUIRED rather than derived: it is the only way this tool
// can enforce the "sidecar never enters the repository" refusal at all.
guard let root = repoRoot, !root.isEmpty else {
    fail("--repo-root <path> is required so the output directory can be refused "
         + "when it resolves inside the repository")
}
guard let video = videoPath, !video.isEmpty else { fail("a video path argument is required") }

let repoURL = URL(fileURLWithPath: root).resolvingSymlinksInPath().standardizedFileURL
let outURL = URL(fileURLWithPath: out, isDirectory: true)
    .resolvingSymlinksInPath().standardizedFileURL
var isDir: ObjCBool = false
guard FileManager.default.fileExists(atPath: outURL.path, isDirectory: &isDir), isDir.boolValue else {
    fail("--out must name an existing directory (resolved: \(outURL.path))")
}
let rootPrefix = repoURL.path.hasSuffix("/") ? repoURL.path : repoURL.path + "/"
if outURL.path == repoURL.path || outURL.path.hasPrefix(rootPrefix) {
    fail("--out resolves INSIDE the repository (\(outURL.path)). The sidecar is a "
         + "derivative of personal footage and must live outside the tree, like the videos.")
}

let videoURL = URL(fileURLWithPath: video)
guard FileManager.default.fileExists(atPath: videoURL.path) else {
    fail("video not found: \(videoURL.path)")
}

// The FILENAME is derived, never operator-supplied.
let sidecarURL = outURL.appendingPathComponent("person_box_sidecar_\(clip).json")

// MARK: - Tool identity

let thisSourcePath = #filePath
guard let thisSource = FileManager.default.contents(atPath: thisSourcePath) else {
    fail("could not read this tool's own source at \(thisSourcePath) for the build binding")
}
let toolSourceSHA = SHA256.hash(data: thisSource).map { String(format: "%02x", $0) }.joined()

let macosProduct = runTool("/usr/bin/sw_vers", ["-productVersion"])
let macosBuild = runTool("/usr/bin/sw_vers", ["-buildVersion"])
let xcodeVersion = runTool("/usr/bin/xcodebuild", ["-version"])
let swiftVersion = runTool("/usr/bin/xcrun", ["swiftc", "--version"])

log("clip=\(clip) tool_source_sha256=\(toolSourceSHA)")
log("macos product=\(macosProduct) build=\(macosBuild)")
log("swift=\(swiftVersion)")
log("xcode=\(xcodeVersion)")

// MARK: - Video identity

guard let videoData = try? Data(contentsOf: videoURL, options: .mappedIfSafe) else {
    fail("could not read \(videoURL.path)")
}
let videoSHA = SHA256.hash(data: videoData).map { String(format: "%02x", $0) }.joined()
let videoBytes = videoData.count
log("video bytes=\(videoBytes) sha256=\(videoSHA)")

// MARK: - Main

struct FrameRecord {
    let i: Int
    let pts: Double
    let status: String
    let decodedWidth: Int
    let decodedHeight: Int
    let sourceHash: UInt64
    let lumaMean: Double
    let lumaSD: Double
    let box: CGRect?
    let confidence: Double?
    let observations: Int?
    let error: String?
}

let semaphore = DispatchSemaphore(value: 0)
var exitCode: Int32 = 0

Task {
    defer { semaphore.signal() }

    let asset = AVURLAsset(url: videoURL)

    // `FrameSource.VideoDecoder.duration()` (:444-448): the isNumeric / > 0
    // guard neither existing prototype applied.
    guard let cmDuration = try? await asset.load(.duration),
          cmDuration.isNumeric, cmDuration.seconds > 0 else {
        exitCode = 1
        FileHandle.standardError.write(Data("PERSON-BOX-SIDECAR-FAIL zero or non-numeric duration\n".utf8))
        return
    }
    let duration = cmDuration.seconds

    // `FrameSource.VideoDecoder.nominalFrameRate()` (:454-462): Float promoted
    // to Double, THEN clamped.
    var rawRate = kAssumedFrameRateWhenUnknown
    var natural = CGSize.zero
    if let track = try? await asset.loadTracks(withMediaType: .video).first {
        if let r = try? await track.load(.nominalFrameRate) { rawRate = Double(r) }
        if let s = try? await track.load(.naturalSize) { natural = s }
    }
    let plan = makeSamplingPlan(duration: duration, nominalFrameRate: rawRate)

    // `FrameSource.maximumDecodedSize(forVideoAt:)` (:288-304) reads the SAME
    // three facts and clamps the rate the same way before sizing the cap.
    let capRate = sanitisedFrameRate(rawRate)
    let cap = maximumDecodedSize(naturalSize: natural, duration: duration,
                                 nominalFrameRate: capRate)

    log("plan duration=\(g17(duration)) fps=\(g17(plan.fps)) step=\(g17(plan.step)) "
        + "start=\(g17(plan.start)) wanted=\(plan.wanted) available=\(plan.available) "
        + "count=\(plan.count) natural=\(Int(natural.width))x\(Int(natural.height)) "
        + "max_size=\(Int(cap.width))x\(Int(cap.height))")

    // `FrameSource.VideoDecoder.makeGenerator` (:429-442).
    let generator = AVAssetImageGenerator(asset: asset)
    generator.appliesPreferredTrackTransform = true
    generator.requestedTimeToleranceBefore = .zero
    generator.requestedTimeToleranceAfter = .zero
    generator.maximumSize = cap

    var records: [FrameRecord] = []
    var revisionUsed = 0
    var supportedRevisions: [Int] = []
    var counts = ["found": 0, "no_observation": 0, "perform_threw": 0]

    for (i, t) in plan.timestamps.enumerated() {
        let time = CMTime(seconds: t, preferredTimescale: kPreferredTimescale)
        guard let cg = try? generator.copyCGImage(at: time, actualTime: nil) else {
            // NOT an E1 cause. The Simulator's decode is measured healthy, so a
            // host decode failure is a defect of the HOST environment, not a
            // property of the clip: writing it into `excluded=[i:fallbackBBox]`
            // would relabel it as a Vision whole-image fallback and eat one slot
            // against the adequacy floor. No sidecar is written at all.
            exitCode = 1
            let msg = "PERSON-BOX-SIDECAR-FAIL decode failed at frame \(i) t=\(g17(t)); "
                + "NO sidecar written (a host decode failure is not an E1 cause)\n"
            FileHandle.standardError.write(Data(msg.utf8))
            return
        }

        let pixels = sourcePixels(cg)
        let hash = fnv1aBytes(pixels)
        let luma = luminanceStats(pixels)

        // Same request configuration production uses: BARE
        // VNDetectHumanRectanglesRequest, upperBodyOnly = false, NO revision pin
        // (SAM3DPoseEstimator.makePersonRectangleRequest, :428-432).
        let request = VNDetectHumanRectanglesRequest()
        request.upperBodyOnly = false
        revisionUsed = request.revision
        supportedRevisions = Array(type(of: request).supportedRevisions).sorted()

        // The generator's frames are `UIImage(cgImage:)`, i.e. `.up`, and
        // production passes `cgOrientation(for: uiImage.imageOrientation)`
        // (:439-441). `.up` is passed EXPLICITLY here.
        let handler = VNImageRequestHandler(cgImage: cg, orientation: .up, options: [:])
        var status = "found"
        var box: CGRect?
        var confidence: Double?
        var observations: Int?
        var errorText: String?
        do {
            try handler.perform([request])
            let obs = (request.results as? [VNHumanObservation]) ?? []
            if let best = obs.max(by: { $0.confidence < $1.confidence }) {
                box = best.boundingBox            // RAW: bottom-left origin, Y-up. NO flip here.
                confidence = Double(best.confidence)
                observations = obs.count
            } else {
                status = "no_observation"
            }
        } catch {
            status = "perform_threw"
            errorText = sanitizedReceipt(error.localizedDescription)
        }
        counts[status, default: 0] += 1

        records.append(FrameRecord(i: i, pts: t, status: status,
                                   decodedWidth: cg.width, decodedHeight: cg.height,
                                   sourceHash: hash, lumaMean: luma.mean, lumaSD: luma.sd,
                                   box: box, confidence: confidence,
                                   observations: observations, error: errorText))
        if i % 10 == 0 || i == plan.count - 1 {
            log("frame=\(i)/\(plan.count) status=\(status) decoded=\(cg.width)x\(cg.height) "
                + (box.map { String(format: "box=%.6f,%.6f,%.6f,%.6f conf=%.4f",
                                    $0.origin.x, $0.origin.y, $0.width, $0.height,
                                    confidence ?? 0) } ?? ""))
        }
    }

    // MARK: - Emit

    var json = "{\n"
    json += "  \"schema\": \(jsonString(kSchema)),\n"
    json += "  \"clip\": \(jsonString(clip)),\n"
    json += "  \"tool\": {\n"
    json += "    \"name\": \(jsonString("tools/pose_fixture/person_box_sidecar")),\n"
    json += "    \"source_sha256\": \(jsonString(toolSourceSHA)),\n"
    json += "    \"swift_version\": \(jsonString(swiftVersion)),\n"
    json += "    \"log_path\": \(jsonString(ProcessInfo.processInfo.environment["PERSON_BOX_SIDECAR_LOG"] ?? "unset"))\n"
    json += "  },\n"
    json += "  \"environment\": {\n"
    json += "    \"macos_product_version\": \(jsonString(macosProduct)),\n"
    json += "    \"macos_build_version\": \(jsonString(macosBuild)),\n"
    json += "    \"xcode_version\": \(jsonString(xcodeVersion))\n"
    json += "  },\n"
    json += "  \"video\": { \"sha256\": \(jsonString(videoSHA)), \"bytes\": \(videoBytes), "
    json += "\"natural_width\": \(Int(natural.width)), \"natural_height\": \(Int(natural.height)) },\n"
    json += "  \"request\": {\n"
    json += "    \"class\": \"VNDetectHumanRectanglesRequest\",\n"
    json += "    \"upper_body_only\": false,\n"
    json += "    \"revision_pinned\": false,\n"
    json += "    \"handler_orientation\": \"up\",\n"
    json += "    \"revision_used\": \(revisionUsed),\n"
    json += "    \"supported_revisions\": [\(supportedRevisions.map(String.init).joined(separator: ", "))]\n"
    json += "  },\n"
    json += "  \"decode\": {\n"
    json += "    \"applies_preferred_track_transform\": true,\n"
    json += "    \"requested_time_tolerance_before\": \"zero\",\n"
    json += "    \"requested_time_tolerance_after\": \"zero\",\n"
    json += "    \"preferred_timescale\": \(kPreferredTimescale),\n"
    json += "    \"maximum_size_w\": \(Int(cap.width)), \"maximum_size_h\": \(Int(cap.height))\n"
    json += "  },\n"
    json += "  \"sampling_plan\": {\n"
    json += "    \"mode\": \"nativeWindow\",\n"
    json += "    \"seconds\": \(kAnalysisWindowSeconds),\n"
    json += "    \"cap_max_native_window_frames\": \(kMaxNativeWindowFrames),\n"
    json += "    \"duration_seconds_bits\": \(jsonString(bits(plan.duration))), "
    json += "\"duration_seconds\": \(jsonString(g17(plan.duration))),\n"
    json += "    \"fps_bits\": \(jsonString(bits(plan.fps))), \"fps\": \(jsonString(g17(plan.fps))),\n"
    json += "    \"step_bits\": \(jsonString(bits(plan.step))), \"step\": \(jsonString(g17(plan.step))),\n"
    json += "    \"start_bits\": \(jsonString(bits(plan.start))), \"start\": \(jsonString(g17(plan.start))),\n"
    json += "    \"wanted\": \(plan.wanted), \"available\": \(plan.available), \"count\": \(plan.count)\n"
    json += "  },\n"
    json += "  \"bbox_space\": \(jsonString(kBBoxSpace)),\n"
    json += "  \"source_hash_algorithm\": \(jsonString(kSourceHashAlgorithm)),\n"
    json += "  \"frames\": [\n"
    for (n, r) in records.enumerated() {
        var f = "    { \"i\": \(r.i), \"pts_bits\": \(jsonString(bits(r.pts))), "
        f += "\"pts\": \(jsonString(String(format: "%.9f", r.pts))), "
        f += "\"status\": \(jsonString(r.status)), "
        f += "\"decoded_width\": \(r.decodedWidth), \"decoded_height\": \(r.decodedHeight), "
        f += "\"pixel_bytes\": \(r.decodedWidth * r.decodedHeight * 4), "
        f += "\"source_hash\": \(jsonString(String(r.sourceHash))), "
        f += "\"luma_mean\": \(g17(r.lumaMean)), \"luma_sd\": \(g17(r.lumaSD))"
        if let b = r.box {
            f += ", \"x_bits\": \(jsonString(bits(Double(b.origin.x)))), "
            f += "\"y_bits\": \(jsonString(bits(Double(b.origin.y)))), "
            f += "\"w_bits\": \(jsonString(bits(Double(b.width)))), "
            f += "\"h_bits\": \(jsonString(bits(Double(b.height)))), "
            f += "\"x\": \(g17(Double(b.origin.x))), \"y\": \(g17(Double(b.origin.y))), "
            f += "\"w\": \(g17(Double(b.width))), \"h\": \(g17(Double(b.height))), "
            f += "\"confidence\": \(g17(r.confidence ?? 0)), "
            f += "\"observations\": \(r.observations ?? 0)"
        }
        if let e = r.error { f += ", \"error\": \(jsonString(e))" }
        f += " }"
        if n < records.count - 1 { f += "," }
        json += f + "\n"
    }
    json += "  ],\n"
    json += "  \"summary\": { \"count\": \(records.count), "
    json += "\"found\": \(counts["found"] ?? 0), "
    json += "\"no_observation\": \(counts["no_observation"] ?? 0), "
    json += "\"perform_threw\": \(counts["perform_threw"] ?? 0) }\n"
    json += "}\n"

    do {
        try json.write(to: sidecarURL, atomically: true, encoding: .utf8)
    } catch {
        exitCode = 1
        FileHandle.standardError.write(Data(
            "PERSON-BOX-SIDECAR-FAIL could not write \(sidecarURL.path): \(error)\n".utf8))
        return
    }

    let written = (try? Data(contentsOf: sidecarURL)) ?? Data()
    let sidecarSHA = SHA256.hash(data: written).map { String(format: "%02x", $0) }.joined()
    log("SUMMARY clip=\(clip) count=\(records.count) found=\(counts["found"] ?? 0) "
        + "no_observation=\(counts["no_observation"] ?? 0) "
        + "perform_threw=\(counts["perform_threw"] ?? 0)")
    log("WROTE path=\(sidecarURL.path) bytes=\(written.count) sha256=\(sidecarSHA)")
}

semaphore.wait()
exit(exitCode)
