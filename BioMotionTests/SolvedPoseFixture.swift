import Foundation
import XCTest
import CryptoKit

@testable import BioMotion

/// Loads `BioMotionTests/Fixtures/solved_pose_video_*.txt` — the RAW per-frame
/// IK joint angles for the two scored gait clips, written by
/// `SolvedPoseFixtureGeneratorTests` and never by hand.
///
/// Same discipline as `GaitClipFixture`: nothing force-unwraps a parsed value,
/// and a malformed file produces an error a test can assert on rather than a
/// trap that takes the whole test target down.
enum SolvedPoseFixture {

    static let formatId = "biomotion-solved-pose-v1"

    struct Fixture {
        let formatId: String
        let clip: String
        let modelSHA256: String
        let dofNames: [String]
        let markerNames: [String]
        let sgTaps: Int
        let frameNumbers: [Int]
        let timestamps: [Double]
        /// `frames[i][j]` is coordinate `j` at frame `i`, radians (metres for the
        /// pelvis translations).
        let frames: [[Double]]
        /// `key=value` tokens of the `sampling_policy` header, empty for a
        /// fixture written before the 2026-08-14 video-driven generator.
        var samplingPolicy: [String: String] = [:]
        /// `key=value` tokens of the `sampling_branch` header, same vintage.
        var samplingBranch: [String: String] = [:]
        /// The `ik_residual_mm_*` headers, keyed by the FULL header name.
        ///
        /// ⚠️ THE FIXTURE FORMAT CARRIES NO PER-FRAME IK RESIDUAL. The
        /// video-driven generator collects `result.markerRMSMeters` per frame
        /// and then writes only THREE CLIP-LEVEL SUMMARIES —
        /// `ik_residual_mm_median`, `_p95`, `_max`
        /// (collected per frame at `SolvedPoseFixtureGeneratorTests.swift:1312`,
        /// reduced at :1328, written at :1460-1462) — while a data
        /// row is `frame t` then one value per DOF and nothing else (re-derived
        /// 2026-08-21: `awk '{print NF}'` on row 0 of both committed fixtures
        /// gives 171 = 1 + 1 + `dofs 169`). Anything that wants a per-frame
        /// residual has to regenerate the fixtures with a new column; the
        /// next-step-51 census therefore records a per-frame POSE-NOISE PROXY
        /// computed in-test and prints these three alongside it, labelled as
        /// the clip-level summaries they are. Empty for a fixture written
        /// before the 2026-08-14 video-driven generator.
        var ikResidualMM: [String: Double] = [:]

        /// Distinct sample intervals, rounded to the microsecond. The registered
        /// clips have exactly one.
        var distinctIntervals: Set<Int> {
            var out = Set<Int>()
            for i in 1..<max(1, timestamps.count) {
                out.insert(Int(((timestamps[i] - timestamps[i - 1]) * 1e6).rounded()))
            }
            return out
        }

        var sampleInterval: Double {
            guard timestamps.count > 1 else { return 0 }
            return (timestamps[timestamps.count - 1] - timestamps[0]) / Double(timestamps.count - 1)
        }
    }

    enum LoadError: Error, CustomStringConvertible {
        case fileNotFound(String)
        case unreadable(String)
        case missingHeader(String)
        case badValue(line: Int, field: Int, text: String)
        case shapeMismatch(String)

        var description: String {
            switch self {
            case .fileNotFound(let n): return "no solved-pose fixture resource named \(n)"
            case .unreadable(let n): return "solved-pose fixture \(n) could not be read"
            case .missingHeader(let k): return "solved-pose fixture is missing header `\(k)`"
            case .badValue(let l, let f, let t):
                return "line \(l) field \(f): `\(t)` is not a plain finite decimal"
            case .shapeMismatch(let m): return "solved-pose fixture shape: \(m)"
            }
        }
    }

    static func modelSHA256(bundle: Bundle) throws -> String {
        guard let path = bundle.path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim") else {
            throw LoadError.fileNotFound("FullBody.osim")
        }
        let data = try Data(contentsOf: URL(fileURLWithPath: path))
        return SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
    }

    static func load(clip: String, bundle: Bundle) throws -> Fixture {
        let resource = "solved_pose_\(clip)"
        guard let url = bundle.url(forResource: resource, withExtension: "txt",
                                   subdirectory: "Fixtures")
                ?? bundle.url(forResource: resource, withExtension: "txt") else {
            throw LoadError.fileNotFound(resource)
        }
        guard let text = try? String(contentsOf: url, encoding: .utf8) else {
            throw LoadError.unreadable(resource)
        }

        var format: String?
        var clipId: String?
        var sha: String?
        var dofNames: [String] = []
        var markerNames: [String] = []
        var taps: Int?
        var declaredFrames: Int?
        var frameNumbers: [Int] = []
        var timestamps: [Double] = []
        var frames: [[Double]] = []
        var samplingPolicy: [String: String] = [:]
        var samplingBranch: [String: String] = [:]
        var ikResidualMM: [String: Double] = [:]

        for (offset, rawLine) in text.split(separator: "\n", omittingEmptySubsequences: false).enumerated() {
            let line = String(rawLine)
            if line.isEmpty || line.hasPrefix("#") { continue }
            let fields = line.split(separator: " ").map(String.init)
            guard let key = fields.first else { continue }
            switch key {
            case "format": format = fields.count > 1 ? fields[1] : nil
            case "clip": clipId = fields.count > 1 ? fields[1] : nil
            case "model_sha256": sha = fields.count > 1 ? fields[1] : nil
            case "dofnames": dofNames = Array(fields.dropFirst())
            case "markers": markerNames = Array(fields.dropFirst())
            case "sg_taps": taps = fields.count > 1 ? Int(fields[1]) : nil
            case "frames": declaredFrames = fields.count > 1 ? Int(fields[1]) : nil
            case "sampling_policy", "sampling_branch":
                // Added 2026-08-14 with the video-driven generator. Every value
                // token is space-free by construction, so the plain field split
                // above already isolates them.
                var kv: [String: String] = [:]
                for token in fields.dropFirst() {
                    let parts = token.split(separator: "=", maxSplits: 1).map(String.init)
                    if parts.count == 2 { kv[parts[0]] = parts[1] }
                }
                if key == "sampling_policy" { samplingPolicy = kv } else { samplingBranch = kv }
            case "ik_residual_mm_median", "ik_residual_mm_p95", "ik_residual_mm_max":
                // Parsed 2026-08-21 for the next-step-51 census, which has to
                // state WHAT IS AVAILABLE where a per-frame residual is not.
                // Read through the same `strictDouble` as every other number in
                // this file, so a malformed summary is simply absent rather
                // than a silent zero.
                if fields.count > 1, let value = strictDouble(fields[1]) { ikResidualMM[key] = value }
            case "generator", "commit", "model", "dofs":
                continue
            default:
                // A data row: frame, t, then one value per DOF.
                guard let frameNumber = Int(key) else { continue }
                guard fields.count >= 2 else {
                    throw LoadError.badValue(line: offset + 1, field: 1, text: line)
                }
                guard let t = strictDouble(fields[1]) else {
                    throw LoadError.badValue(line: offset + 1, field: 2, text: fields[1])
                }
                var row = [Double]()
                row.reserveCapacity(fields.count - 2)
                for (i, field) in fields.dropFirst(2).enumerated() {
                    guard let v = strictDouble(field) else {
                        throw LoadError.badValue(line: offset + 1, field: i + 3, text: field)
                    }
                    row.append(v)
                }
                frameNumbers.append(frameNumber)
                timestamps.append(t)
                frames.append(row)
            }
        }

        guard let format else { throw LoadError.missingHeader("format") }
        guard let clipId else { throw LoadError.missingHeader("clip") }
        guard let sha else { throw LoadError.missingHeader("model_sha256") }
        guard let taps else { throw LoadError.missingHeader("sg_taps") }
        guard !dofNames.isEmpty else { throw LoadError.missingHeader("dofnames") }
        guard !markerNames.isEmpty else { throw LoadError.missingHeader("markers") }
        if let declaredFrames, declaredFrames != frames.count {
            throw LoadError.shapeMismatch("declared \(declaredFrames) frames, found \(frames.count)")
        }
        for row in frames where row.count != dofNames.count {
            throw LoadError.shapeMismatch("a row carries \(row.count) values against \(dofNames.count) DOFs")
        }

        return Fixture(formatId: format, clip: clipId, modelSHA256: sha, dofNames: dofNames,
                       markerNames: markerNames, sgTaps: taps, frameNumbers: frameNumbers,
                       timestamps: timestamps, frames: frames,
                       samplingPolicy: samplingPolicy, samplingBranch: samplingBranch,
                       ikResidualMM: ikResidualMM)
    }

    /// Stricter than `Double(_:)`, which accepts `nan`, `inf` and hex floats —
    /// none of which is a joint angle.
    private static func strictDouble(_ text: String) -> Double? {
        guard !text.isEmpty else { return nil }
        var seenDigit = false, seenDot = false
        for (i, c) in text.enumerated() {
            if c == "-" || c == "+" { if i != 0 { return nil }; continue }
            if c == "." { if seenDot { return nil }; seenDot = true; continue }
            guard c.isASCII, c.isNumber else { return nil }
            seenDigit = true
        }
        guard seenDigit, let value = Double(text), value.isFinite else { return nil }
        return value
    }
}

// MARK: - The shared clip traversal

extension MuscleLengthModeTests {

    /// Builds the ONE per-clip traversal G2, G3(iv), G5, G7 and G8 all read.
    ///
    /// Full frame, no subsampling. The Savitzky-Golay stage runs HERE, in-test,
    /// from the stored raw series at the production taps — only IK left the lane.
    static func buildTraversal(clip: String, ctx: ModelContext) throws -> ClipTraversal {
        let wall = Date()
        let out = ClipTraversal()
        let bundle = Bundle(for: MuscleLengthModeTests.self)
        let fixture = try SolvedPoseFixture.load(clip: clip, bundle: bundle)

        // --- DIAGNOSTIC, STATUS next-step 51. NEVER A GATE. ------------------
        // Registered as a stored reference BEFORE anything can throw, so a clip
        // that dies in the runtime-span guard leaves an EMPTY census that the
        // report prints as such, rather than no census at all that a reader
        // could mistake for "no flips". NOTHING in the census writes to `out`,
        // and no counted number is read back out of it.
        let census = MuscleModeFlipCensus(clip: clip, ikResidualMM: fixture.ikResidualMM)
        muscleModeFlipCensusLock.lock()
        muscleModeFlipCensusByClip[clip] = census
        muscleModeFlipCensusLock.unlock()

        let dofCount = fixture.dofNames.count
        let taps = MuscleLengthModeClassifier.taps
        let halfWindow = taps / 2

        // --- SG, in-test, per DOF, pushed on EVERY frame ---------------------
        var smoothedQ: [[Double]] = []
        var smoothedDQ: [[Double]] = []
        var centres: [Double] = []
        var filters = (0..<dofCount).map { _ in WindowedDerivativeFilter(taps: taps) }
        for (i, pose) in fixture.frames.enumerated() {
            var q = [Double](), dq = [Double]()
            var centre = fixture.timestamps[i]
            var warm = true
            for j in 0..<dofCount {
                if let o = filters[j].push(pose[j], timestamp: fixture.timestamps[i]) {
                    q.append(o.pos); dq.append(o.vel); centre = o.center
                } else { warm = false }
            }
            guard warm, q.count == dofCount else { continue }
            smoothedQ.append(q); smoothedDQ.append(dq); centres.append(centre)
        }
        out.warmedCount = smoothedQ.count

        let filter = WindowedDerivativeFilter(taps: taps)
        let c0 = filter.posCoefficients[halfWindow]
        let dt = fixture.sampleInterval

        // --- sigmaHat, clip face: MAD of the SG POSITION residual ------------
        var sigmaHat = [Double](repeating: 0, count: dofCount)
        for j in 0..<dofCount {
            var residuals: [Double] = []
            residuals.reserveCapacity(out.warmedCount)
            for w in 0..<out.warmedCount {
                residuals.append(fixture.frames[w + halfWindow][j] - smoothedQ[w][j])
            }
            sigmaHat[j] = MuscleLengthModeClassifier.robustJointNoiseRadians(
                residuals: residuals, centreCoefficient: c0)
        }

        // --- Rule 2: unmeasured coordinates over the warmed RAW poses --------
        let warmedRaw = (0..<out.warmedCount).map { fixture.frames[$0 + halfWindow] }
        let unmeasured = MuscleObservabilityMask.unmeasuredCoordinates(
            warmedPoses: warmedRaw, coordinateCount: dofCount)

        // --- Pass 1: drive-aware identifiability at every warmed frame -------
        // The lower-limb block is the model's own leading coordinate run, taken
        // from the reference fixture's coordinate order rather than named here.
        let lowerLimb = Set(ctx.table.coordinateNames.prefix(20)
            .compactMap { ctx.dofNames.firstIndex(of: $0) })
        var perFrameNullFractions: [[Double]] = []
        let jacobianStart = Date()
        for w in 0..<out.warmedCount {
            guard ctx.setPose(smoothedQ[w]),
                  let fractions = nullFractions(ctx: ctx, markers: fixture.markerNames) else {
                XCTFail("the marker Jacobian did not resolve on \(clip) warmed frame \(w)")
                continue
            }
            perFrameNullFractions.append(fractions)
            let unidentifiedLowerLimb = lowerLimb.filter {
                !MuscleObservabilityMask.isIdentified(nullFraction: fractions[$0])
            }
            if unidentifiedLowerLimb.isEmpty { out.framesWithEmptyUnidentifiedLowerLimb += 1 }
        }
        out.msPerFrameIdentifiability = out.warmedCount == 0 ? 0
            : Date().timeIntervalSince(jacobianStart) * 1000.0 / Double(out.warmedCount)

        let identified = MuscleObservabilityMask.clipIdentifiedCoordinates(
            perFrameNullFractions: perFrameNullFractions, coordinateCount: dofCount)
        out.clipUnidentifiedLowerLimb = Set(lowerLimb.subtracting(identified).map { ctx.dofNames[$0] })

        // --- Runtime spans for the displayed set -----------------------------
        guard out.warmedCount > 0,
              ctx.setPose(smoothedQ[0]),
              let spanRows = ctx.momentArms(pose: smoothedQ[0], muscles: ctx.displayedMuscles,
                                            coordinates: ctx.dofNames) else {
            throw NSError(domain: "MuscleLengthModeTests", code: 3,
                          userInfo: [NSLocalizedDescriptionKey: "runtime spans unavailable on \(clip)"])
        }
        var spanByMuscle: [String: Set<Int>] = [:]
        for (m, name) in ctx.displayedMuscles.enumerated() {
            spanByMuscle[name] = MuscleObservabilityMask.spannedCoordinates(momentArmRow: spanRows[m])
        }

        // --- Rules 0+1+2: which capsules are admitted ------------------------
        var capsuleSpans: [String: Set<Int>] = [:]
        for resolution in ctx.resolutions where resolution.isResolved {
            capsuleSpans[resolution.capsule] = resolution.modelMuscles.reduce(into: Set<Int>()) {
                $0.formUnion(spanByMuscle[$1] ?? [])
            }
        }
        let hipPrefix = "hip_"
        out.hipCapsules = capsuleSpans
            .filter { $0.value.contains { ctx.dofNames[$0].hasPrefix(hipPrefix) } }
            .keys.sorted()
        var admitted: Set<String> = []
        for (capsule, spanned) in capsuleSpans {
            if MuscleObservabilityMask.isSuppressed(spanned: spanned, identified: identified,
                                                    unmeasured: unmeasured) {
                if out.hipCapsules.contains(capsule) { out.suppressedHipCapsules.append(capsule) }
            } else {
                admitted.insert(capsule)
            }
        }
        out.suppressedHipCapsules.sort()
        out.admittedCapsules = admitted
        let admittedResolutions = ctx.resolutions.filter { admitted.contains($0.capsule) }
        out.admittedMuscles = admittedResolutions.flatMap(\.modelMuscles).sorted()

        // --- The stencil: the union of the DISPLAYED set's spans -------------
        var stencilIndices: [Int] = []
        var seen = Set<Int>()
        for name in ctx.displayedMuscles {
            for j in (spanByMuscle[name] ?? []).sorted() where seen.insert(j).inserted {
                stencilIndices.append(j)
            }
        }
        stencilIndices.sort()
        out.stencilCoordinates = stencilIndices.map { ctx.dofNames[$0] }

        // --- DIAGNOSTIC: the per-frame pose-noise reading (next-step 51) ------
        // The fixture has NO per-frame IK residual — only the three clip-level
        // summaries (see `Fixture.ikResidualMM`) — so the per-frame quantity the
        // census records at a flip is the SG POSITION RESIDUAL the noise
        // estimate is itself built from, `|raw − SG| / σ̂ⱼ`, over the stencil
        // coordinates. Same bytes as the `sigmaHat` loop above, one z-score per
        // coordinate instead of one MAD per coordinate. σ̂ⱼ = 0 means that
        // coordinate never moves off its SG fit at all, so its residual carries
        // no information and it is skipped rather than dividing by zero.
        // Computed OUTSIDE both timed blocks, so neither of G5's per-frame cost
        // receipts is charged for it.
        var residualZMaxByFrame = [Double](repeating: 0, count: out.warmedCount)
        var residualZRMSByFrame = [Double](repeating: 0, count: out.warmedCount)
        for w in 0..<out.warmedCount {
            var worst = 0.0, sumSquares = 0.0, counted = 0
            for j in stencilIndices where sigmaHat[j] > 0 {
                let z = abs(fixture.frames[w + halfWindow][j] - smoothedQ[w][j]) / sigmaHat[j]
                worst = max(worst, z)
                sumSquares += z * z
                counted += 1
            }
            residualZMaxByFrame[w] = worst
            residualZRMSByFrame[w] = counted == 0 ? 0 : (sumSquares / Double(counted)).squareRoot()
        }
        census.baselineResidualZMax = residualZMaxByFrame
        census.baselineResidualZRMS = residualZRMSByFrame

        // --- Pass 2: R, L, mode, witness B -----------------------------------
        // The block is the DISPLAYED set, not the admitted one: compute is paid
        // for every muscle the picture would draw, so that is the cost G5 has to
        // record, and it stays measurable when the mask admits nothing.
        let stencilNames = out.stencilCoordinates
        let stencilNoise = stencilIndices.map { sigmaHat[$0] }
        let displayed = ctx.displayedMuscles
        let displayedIndices = ctx.indices(displayed)
        let admittedSet = Set(out.admittedMuscles)
        let muscles = displayed
        let muscleIndices = displayedIndices
        var modeSeries: [String: [MuscleLengthMode]] = [:]
        var lengthSeries: [String: [Double]] = [:]
        var previousLengths: [String: Double] = [:]
        var previousSignatures: [String: UInt64] = [:]
        var previousDiffDeadband: [String: Double] = [:]
        var previousRate: [String: Double] = [:]

        // --- DIAGNOSTIC side tables (next-step 51) ---------------------------
        // FLAT and PREALLOCATED on purpose: the census costs two array stores
        // per muscle-frame, not two dictionary hashes, so G5's `ms_per_frame_mode`
        // receipt is not materially charged for an instrument that no gate reads.
        // `diagSeriesFrame` exists because a frame whose moment arms fail to
        // resolve is `continue`d out of the loop below WITHOUT appending to
        // `modeSeries` — so a series index is NOT a warmed-frame index, and a
        // census that assumed it were would name the wrong frame.
        let diagStride = max(1, out.warmedCount)
        var diagRate = [Double](repeating: 0, count: muscles.count * diagStride)
        var diagDeadband = [Double](repeating: 0, count: muscles.count * diagStride)
        var diagIndexByMuscle: [String: Int] = [:]
        for (m, name) in muscles.enumerated() { diagIndexByMuscle[name] = m }
        var diagSeriesFrame: [Int] = []
        var diagSeriesRule3: [Bool] = []

        let modeStart = Date()

        for w in 0..<out.warmedCount {
            guard let rows = ctx.momentArms(pose: smoothedQ[w], muscles: muscles,
                                            coordinates: stencilNames) else { continue }
            let rule3 = ctx.computer.lastUnresolvedDiscontinuitySamples > 0
            let diagSeriesIndex = diagSeriesFrame.count
            diagSeriesFrame.append(w)
            diagSeriesRule3.append(rule3)
            var signatures: NSArray?
            let lengths = ctx.computer.muscleLengths(forIndices: muscleIndices,
                                                     signatures: &signatures)
            let dq = stencilIndices.map { smoothedDQ[w][$0] }

            for (m, name) in muscles.enumerated() {
                let isAdmitted = admittedSet.contains(name)
                let rate = MuscleLengthModeClassifier.lengthRate(momentArmRow: rows[m],
                                                                 jointVelocity: dq)
                let deadband = MuscleLengthModeClassifier.rateDeadbandMetresPerSecond(
                    momentArmRow: rows[m], jointNoiseRadians: stencilNoise,
                    velocityNoiseGain: velocityNoiseGain, sampleInterval: dt)
                let mode = rule3 ? .indeterminate
                    : MuscleLengthModeClassifier.classify(value: rate, deadband: deadband)
                // DIAGNOSTIC: the two numbers `classify` just compared, kept so
                // the census can report `|v| / D` at a flip. Written for the
                // DISPLAYED set, which is the set `modeSeries` is keyed by.
                if diagSeriesIndex < diagStride {
                    diagRate[m * diagStride + diagSeriesIndex] = rate
                    diagDeadband[m * diagStride + diagSeriesIndex] = deadband
                }
                modeSeries[name, default: []].append(mode)
                if rule3 && isAdmitted { out.rule3Excluded += 1 }

                guard isAdmitted, let lengths, m < lengths.count else { continue }
                let length = lengths[m].doubleValue
                lengthSeries[name, default: []].append(length)
                let signature = (signatures?[m] as? NSNumber)?.uint64Value ?? 0
                if let prior = previousSignatures[name], prior != signature {
                    out.signatureChanges += 1
                }
                let diffDeadband = MuscleLengthModeClassifier.differenceDeadbandMetres(
                    momentArmRow: rows[m], jointNoiseRadians: stencilNoise, centreCoefficient: c0)

                // Witness A is scored against D_rate at the SAME frame; witness B
                // is the raw difference of two smoothed lengths against D_diff.
                if !rule3, let priorLength = previousLengths[name],
                   let priorDeadband = previousDiffDeadband[name],
                   let priorRateValue = previousRate[name] {
                    _ = priorRateValue
                    let difference = length - priorLength
                    let bBand = max(diffDeadband, priorDeadband)
                    if abs(rate) > deadband && abs(difference) > bBand {
                        out.witnessJointFrames += 1
                        if (rate > 0) == (difference > 0) { out.witnessAgreements += 1 }
                    }
                }
                previousLengths[name] = length
                previousSignatures[name] = signature
                previousDiffDeadband[name] = diffDeadband
                previousRate[name] = rate
            }
        }
        out.msPerFrameMode = out.warmedCount == 0 ? 0
            : Date().timeIntervalSince(modeStart) * 1000.0 / Double(out.warmedCount)

        guard !out.admittedMuscles.isEmpty else {
            out.minimumLengthRange = 0
            out.traversalSeconds = Date().timeIntervalSince(wall)
            return out
        }

        // --- Rule 4: capsule modes, then the G2 statistics -------------------
        var capsuleSeries: [String: [MuscleLengthMode]] = [:]
        // DIAGNOSTIC: the SAME filter `heads` applies, kept as NAMES so the
        // census can say which head broke unanimity. `compactMap` above drops a
        // head with no series; `filter { modeSeries[$0] != nil }` drops exactly
        // the same ones in exactly the same order.
        var capsuleHeads: [String: [String]] = [:]
        for resolution in admittedResolutions {
            let heads = resolution.modelMuscles.compactMap { modeSeries[$0] }
            guard let first = heads.first else { continue }
            var series: [MuscleLengthMode] = []
            for w in 0..<first.count {
                series.append(MuscleObservabilityMask.unanimousMode(heads.map { $0[w] }))
            }
            capsuleSeries[resolution.capsule] = series
            capsuleHeads[resolution.capsule] = resolution.modelMuscles.filter { modeSeries[$0] != nil }
        }

        // --- DIAGNOSTIC: the flip recorder (next-step 51) ---------------------
        // Called AFTER each existing counter increments, inside the same `if`,
        // reading only what that counter already decided. It adds no branch to
        // the counting logic and cannot move `flickerCentres`, `flickerDenominator`,
        // `greyTransitions` or `greyDenominator` — the four numbers G2(a)/(e)
        // are pinned on.
        func recordFlip(capsule: String, series: [MuscleLengthMode], t: Int, kind: String) {
            var heads: [MuscleModeFlipEvent.Head] = []
            var breakers: [MuscleModeFlipEvent.Head] = []
            for head in capsuleHeads[capsule] ?? [] {
                guard let headSeries = modeSeries[head], t < headSeries.count,
                      let m = diagIndexByMuscle[head], t < diagStride else { continue }
                let entry = MuscleModeFlipEvent.Head(
                    name: head,
                    before: headSeries[t - 1],
                    after: headSeries[t],
                    ratioBefore: MuscleModeFlipCensus.ratio(value: diagRate[m * diagStride + t - 1],
                                                            deadband: diagDeadband[m * diagStride + t - 1]),
                    ratio: MuscleModeFlipCensus.ratio(value: diagRate[m * diagStride + t],
                                                      deadband: diagDeadband[m * diagStride + t]))
                heads.append(entry)
                if entry.after != entry.before { breakers.append(entry) }
            }
            let frame = t < diagSeriesFrame.count ? diagSeriesFrame[t] : -1
            let previousFrame = t - 1 < diagSeriesFrame.count ? diagSeriesFrame[t - 1] : -1
            census.record(MuscleModeFlipEvent(
                capsule: capsule,
                kind: kind,
                seriesIndex: t,
                warmedFrame: frame,
                before: series[t - 1],
                after: series[t],
                rule3Before: t - 1 < diagSeriesRule3.count ? diagSeriesRule3[t - 1] : false,
                rule3After: t < diagSeriesRule3.count ? diagSeriesRule3[t] : false,
                residualZMax: frame >= 0 && frame < residualZMaxByFrame.count
                    ? residualZMaxByFrame[frame] : -1,
                residualZRMS: frame >= 0 && frame < residualZRMSByFrame.count
                    ? residualZRMSByFrame[frame] : -1,
                residualZMaxBefore: previousFrame >= 0 && previousFrame < residualZMaxByFrame.count
                    ? residualZMaxByFrame[previousFrame] : -1,
                heads: heads,
                breakers: breakers))
        }

        for (capsule, series) in capsuleSeries {
            var defined = 0, directional = 0
            for mode in series {
                out.totalSamples += 1
                if mode.isDefined { defined += 1; out.definedSamples += 1 }
                if mode.isDirectional { directional += 1; out.directionalSamples += 1 }
            }
            out.perCapsuleDirectionalFraction[capsule] = defined == 0 ? 0
                : Double(directional) / Double(defined)

            // Flicker: t is a centre iff mode(t) != mode(t-1) and the run
            // reverts within two samples, over samples where t-1..t+2 are all
            // DEFINED. Grey transitions are counted separately, because that
            // metric excludes them by construction.
            for t in 1..<max(1, series.count) {
                if t + 2 < series.count,
                   series[t - 1].isDefined, series[t].isDefined,
                   series[t + 1].isDefined, series[t + 2].isDefined {
                    out.flickerDenominator += 1
                    if series[t] != series[t - 1] {
                        for tPrime in (t + 1)...(t + 2) where tPrime < series.count {
                            if series[tPrime] == series[t - 1],
                               (t + 1..<tPrime).allSatisfy({ series[$0] == series[t] }) {
                                out.flickerCentres += 1
                                recordFlip(capsule: capsule, series: series, t: t, kind: "flicker")
                                break
                            }
                        }
                    }
                }
                out.greyDenominator += 1
                if series[t].isDefined != series[t - 1].isDefined {
                    out.greyTransitions += 1
                    recordFlip(capsule: capsule, series: series, t: t,
                               kind: series[t].isDefined ? "grey_leave" : "grey_enter")
                }
            }
        }

        // --- G7(b) sentinel and G8 trend -------------------------------------
        for (name, series) in lengthSeries {
            guard series.count > 2 else { continue }
            let lo = series.min() ?? 0, hi = series.max() ?? 0
            out.minimumLengthRange = min(out.minimumLengthRange, hi - lo)
            let n = Double(series.count)
            let meanX = (n - 1) / 2
            let meanY = series.reduce(0, +) / n
            var sxy = 0.0, sxx = 0.0
            for (i, y) in series.enumerated() {
                let dx = Double(i) - meanX
                sxy += dx * (y - meanY)
                sxx += dx * dx
            }
            let slope = sxx == 0 ? 0 : sxy / sxx
            let excursion = abs(slope) * (n - 1)
            let range = hi - lo
            out.trendExcursions[name] = range > 0 ? excursion / range : .infinity
        }
        if out.minimumLengthRange == .infinity { out.minimumLengthRange = 0 }

        // Re-impose a stored pose from the middle of the clip and require the
        // same L_MT vector back. A stale skeleton is smooth, non-flickering and
        // wrong, and nothing else in the battery sees it.
        let midpoint = out.warmedCount / 2
        if ctx.setPose(smoothedQ[midpoint]) {
            var ignored: NSArray?
            let first = ctx.computer.muscleLengths(forIndices: muscleIndices, signatures: &ignored)
            _ = ctx.setPose(smoothedQ[0])
            _ = ctx.setPose(smoothedQ[midpoint])
            let second = ctx.computer.muscleLengths(forIndices: muscleIndices, signatures: &ignored)
            if let first, let second, first.count == second.count {
                for i in 0..<first.count {
                    out.reimposedPoseMaxDelta = max(out.reimposedPoseMaxDelta,
                                                    abs(first[i].doubleValue - second[i].doubleValue))
                }
            }
        }

        out.traversalSeconds = Date().timeIntervalSince(wall)
        return out
    }

    static var velocityNoiseGain: Double {
        WindowedDerivativeFilter.velocityNoiseGain(taps: MuscleLengthModeClassifier.taps)
    }
}

// MARK: - The next-step-51 flicker / grey census (DIAGNOSTIC, NEVER A GATE)

/// One flicker centre or one grey transition, carrying the four facts STATUS
/// next-step 51 demands be MEASURED before anybody proposes a fix for the
/// G2(a)/G2(e) failures: WHICH capsule, WHICH muscle head broke unanimity,
/// `|v| / D` at the flip, and the per-frame pose reading at that frame.
///
/// HOW THIS IS READ — the whole reason the instrument exists, written down
/// BEFORE the numbers land so the reading cannot be chosen to suit them:
///
/// * breaker `|v| / D` clustered JUST ABOVE 1.0 ⇒ the flips are heads crossing
///   their own deadband edge, i.e. a **DEADBAND / POSE-NOISE** problem;
/// * breaker `|v| / D` LARGE, and large on BOTH sides of a lengthening↔shortening
///   reversal, ⇒ the length rate genuinely reversed and the layer is reporting
///   what it was given, i.e. a **CLASSIFIER / KINEMATICS** problem;
/// * `rule3=1` at the flip ⇒ NEITHER of those: that frame was forced
///   `.indeterminate` by an unresolved wrap switch, which is a fourth cause and
///   is counted apart so it cannot be silently attributed to the other three.
///
/// NO THRESHOLD IS ATTACHED TO ANY OF THIS AND NONE MAY BE. A bar written now
/// would be a bar selected against an already-known 4.8 %/5.6 % failure, which
/// is the same move as editing one. This type is read by a `print` and by
/// nothing else: no gate, no assertion, no pinned number, and nothing on
/// `ClipTraversal`.
struct MuscleModeFlipEvent {

    /// One head of the capsule at the flip, with BOTH ratios — a flip whose
    /// breaker sits at 1.05 after and 0.98 before is a deadband crossing; one at
    /// 8.0 after and 6.0 before with opposite signs is a real reversal.
    struct Head {
        let name: String
        let before: MuscleLengthMode
        let after: MuscleLengthMode
        /// `|v| / D` for this head at `t − 1`, from the two numbers `classify`
        /// itself compared at that frame.
        let ratioBefore: Double
        /// `|v| / D` for this head at `t`, the flip frame.
        let ratio: Double
    }

    let capsule: String
    /// `flicker`, `grey_enter` (defined → indeterminate) or `grey_leave`
    /// (indeterminate → defined).
    let kind: String
    /// Index into the CAPSULE MODE SERIES, which is what the G2 counters walk.
    let seriesIndex: Int
    /// The WARMED-frame index that series index maps to. Not the same number
    /// whenever a frame's moment arms failed to resolve and the traversal
    /// `continue`d past it without appending a mode.
    let warmedFrame: Int
    let before: MuscleLengthMode
    let after: MuscleLengthMode
    let rule3Before: Bool
    let rule3After: Bool
    /// `max |raw − SG| / σ̂` over the stencil at the flip frame. **NOT the IK
    /// residual**: the fixture carries no per-frame IK residual (see
    /// `SolvedPoseFixture.Fixture.ikResidualMM`), so this is the SG position
    /// residual the noise estimate is itself built from. `-1` = unavailable.
    let residualZMax: Double
    let residualZRMS: Double
    /// The same reading one frame earlier, so a spike AT the flip is separable
    /// from a generally noisy neighbourhood.
    let residualZMaxBefore: Double
    let heads: [Head]
    /// Heads whose OWN mode changed across the flip. A capsule's mode is the
    /// UNANIMOUS mode of its heads (`MuscleObservabilityMask.unanimousMode`), so
    /// these are exactly the heads that could have broken or restored unanimity.
    /// An EMPTY breaker list on a recorded flip would mean the capsule mode moved
    /// while no head moved, which is impossible under `unanimousMode` — it is
    /// counted and printed rather than assumed away.
    let breakers: [Head]
}

/// The per-clip census. One instance per clip, created by `buildTraversal`
/// BEFORE anything in it can throw, so an empty census always means "nothing
/// was recorded", never "the traversal never got there".
final class MuscleModeFlipCensus {

    let clip: String
    /// The three CLIP-LEVEL IK residual summaries the fixture actually carries.
    let ikResidualMM: [String: Double]
    /// The stencil pose-noise reading at EVERY warmed frame — the control the
    /// at-flip readings are compared against. Without it, "the residual at the
    /// flip was 3.1" is a number with no scale.
    var baselineResidualZMax: [Double] = []
    var baselineResidualZRMS: [Double] = []
    private(set) var events: [MuscleModeFlipEvent] = []

    init(clip: String, ikResidualMM: [String: Double]) {
        self.clip = clip
        self.ikResidualMM = ikResidualMM
    }

    func record(_ event: MuscleModeFlipEvent) { events.append(event) }

    /// `|v| / D`, the classifier's own comparison expressed as one number.
    /// A non-positive or non-finite deadband yields `.infinity`, which the
    /// report counts and excludes rather than folding into a median.
    static func ratio(value: Double, deadband: Double) -> Double {
        guard value.isFinite, deadband.isFinite, deadband > 0 else { return .infinity }
        return abs(value) / deadband
    }

    /// One character per mode, so a 300-line event census stays readable.
    static func code(_ mode: MuscleLengthMode) -> String {
        switch mode {
        case .lengthening: return "L"
        case .shortening: return "S"
        case .noChangeThisViewCanResolve: return "N"
        case .indeterminate: return "I"
        }
    }

    /// Bucket edges chosen around 1.0 because 1.0 is where `classify` switches,
    /// not because any measurement suggested them.
    static let ratioBucketEdges: [Double] = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0]
    static let ratioBucketLabels = ["lt0.5", "0.5-1", "1-1.5", "1.5-2",
                                    "2-3", "3-5", "5-10", "ge10"]

    static func histogram(_ values: [Double]) -> [Int] {
        var counts = [Int](repeating: 0, count: ratioBucketLabels.count)
        for value in values {
            var bucket = ratioBucketEdges.count
            for (i, edge) in ratioBucketEdges.enumerated() where value < edge {
                bucket = i
                break
            }
            counts[bucket] += 1
        }
        return counts
    }

    private static func histogramText(_ values: [Double]) -> String {
        let counts = histogram(values)
        var parts: [String] = []
        for (i, label) in ratioBucketLabels.enumerated() where i < counts.count {
            parts.append("\(label)=\(counts[i])")
        }
        return parts.joined(separator: " ")
    }

    /// The census as ONE multi-line greppable string, in the file's established
    /// `MODE-METRIC` house style. Aggregates are `MODE-METRIC g2diag `; the raw
    /// per-event rows are `MODE-METRIC g2diag-event `, so `grep 'MODE-METRIC
    /// g2diag'` gets both and `grep 'MODE-METRIC g2diag '` gets the summary only.
    func report() -> String {
        let head = "MODE-METRIC g2diag clip=\(clip)"
        var lines: [String] = []
        lines.append(head + " instrument=next_step_51_flip_census gates=NONE assertions=NONE"
                     + " reading=[breaker_ratio_just_above_1=DEADBAND_OR_POSE_NOISE;"
                     + "breaker_ratio_large_both_sides=CLASSIFIER_OR_KINEMATICS;"
                     + "rule3=UNRESOLVED_WRAP_SWITCH_a_fourth_cause]"
                     + " modes=[L=lengthening,S=shortening,N=noChangeThisViewCanResolve,"
                     + "I=indeterminate]")

        // The file's own median, so the census cannot drift from the estimator
        // the classifier itself uses.
        let median: ([Double]) -> Double = MuscleLengthModeClassifier.median
        lines.append(head + " per_frame_ik_residual=UNAVAILABLE_IN_FIXTURE"
                     + " fixture_carries=clip_level_only"
                     + String(format: " ik_residual_mm_median=%.6f ik_residual_mm_p95=%.6f"
                              + " ik_residual_mm_max=%.6f",
                              ikResidualMM["ik_residual_mm_median"] ?? -1,
                              ikResidualMM["ik_residual_mm_p95"] ?? -1,
                              ikResidualMM["ik_residual_mm_max"] ?? -1)
                     + " substitute=sg_position_residual_z_over_stencil"
                     + " substitute_definition=max_and_rms_of_abs_raw_minus_sg_over_sigma_hat")

        guard !events.isEmpty else {
            lines.append(head + " events=0 VACUOUS-BY-CONSTRUCTION"
                         + " reason=no_flicker_centre_and_no_grey_transition_reached_the_recorder"
                         + " note=an_empty_census_scores_nothing_and_is_not_a_pass")
            return lines.joined(separator: "\n")
        }

        let flicker = events.filter { $0.kind == "flicker" }.count
        let greyEnter = events.filter { $0.kind == "grey_enter" }.count
        let greyLeave = events.filter { $0.kind == "grey_leave" }.count
        let noBreaker = events.filter { $0.breakers.isEmpty }.count
        let multiHead = events.filter { $0.heads.count > 1 }.count
        let wholeCapsuleFlips = events.filter { !$0.heads.isEmpty
            && $0.breakers.count == $0.heads.count }.count
        lines.append(head + " events=\(events.count) flicker=\(flicker)"
                     + " grey_enter=\(greyEnter) grey_leave=\(greyLeave)"
                     + " capsules=\(Set(events.map(\.capsule)).count)"
                     + " multi_head_events=\(multiHead)"
                     + " all_heads_flipped=\(wholeCapsuleFlips)"
                     + " single_head_broke_unanimity=\(events.count - wholeCapsuleFlips - noBreaker)"
                     + " no_breaker=\(noBreaker)"
                     + " rule3_at_flip=\(events.filter(\.rule3After).count)"
                     + " rule3_before_flip=\(events.filter(\.rule3Before).count)")

        let breakerRatios = events.flatMap { $0.breakers.map(\.ratio) }
        let finiteRatios = breakerRatios.filter(\.isFinite)
        let finiteRatiosBefore = events.flatMap { $0.breakers.map(\.ratioBefore) }.filter(\.isFinite)
        lines.append(head + " which=at_flip breakers=\(breakerRatios.count)"
                     + " non_finite=\(breakerRatios.count - finiteRatios.count)"
                     + String(format: " median=%.4f", median(finiteRatios))
                     + " " + Self.histogramText(finiteRatios))
        lines.append(head + " which=one_frame_before breakers=\(finiteRatiosBefore.count)"
                     + String(format: " median=%.4f", median(finiteRatiosBefore))
                     + " " + Self.histogramText(finiteRatiosBefore))

        let atFlipMax = events.map(\.residualZMax).filter { $0 >= 0 }
        let atFlipRMS = events.map(\.residualZRMS).filter { $0 >= 0 }
        lines.append(head + " pose_noise_control frames=\(baselineResidualZMax.count)"
                     + " flip_frames=\(atFlipMax.count)"
                     + String(format: " resid_z_max_median_all=%.4f resid_z_max_median_at_flip=%.4f"
                              + " resid_z_rms_median_all=%.4f resid_z_rms_median_at_flip=%.4f",
                              median(baselineResidualZMax), median(atFlipMax),
                              median(baselineResidualZRMS), median(atFlipRMS))
                     + " note=at_flip_equal_to_all_means_pose_noise_does_not_mark_the_flip_frames")

        for capsule in Set(events.map(\.capsule)).sorted() {
            let own = events.filter { $0.capsule == capsule }
            let ratios = own.flatMap { $0.breakers.map(\.ratio) }.filter(\.isFinite)
            lines.append(head + " capsule=\(capsule) heads=\(own.first?.heads.count ?? 0)"
                         + " events=\(own.count)"
                         + " flicker=\(own.filter { $0.kind == "flicker" }.count)"
                         + " grey=\(own.filter { $0.kind != "flicker" }.count)"
                         + " breakers=\(ratios.count)"
                         + String(format: " median_ratio=%.4f", median(ratios)))
        }

        var breaksByHead: [String: [Double]] = [:]
        for event in events {
            for breaker in event.breakers {
                breaksByHead[breaker.name, default: []].append(breaker.ratio)
            }
        }
        for name in breaksByHead.keys.sorted() {
            let ratios = (breaksByHead[name] ?? []).filter(\.isFinite)
            lines.append(head + " head=\(name) broke_unanimity=\(breaksByHead[name]?.count ?? 0)"
                         + String(format: " median_ratio=%.4f", median(ratios)))
        }

        // The raw rows. Sorted so two runs of the same fixtures print the same
        // bytes in the same order: `capsuleSeries` is a dictionary and its
        // iteration order is not stable across runs.
        let ordered = events.sorted {
            ($0.capsule, $0.seriesIndex, $0.kind) < ($1.capsule, $1.seriesIndex, $1.kind)
        }
        for event in ordered {
            let breakers = event.breakers.map {
                String(format: "%@:%@>%@:%.3f>%.3f", $0.name,
                       Self.code($0.before), Self.code($0.after), $0.ratioBefore, $0.ratio)
            }.joined(separator: ";")
            let others = event.heads.filter { entry in
                !event.breakers.contains { $0.name == entry.name }
            }.map {
                String(format: "%@:%@:%.3f", $0.name, Self.code($0.after), $0.ratio)
            }.joined(separator: ";")
            lines.append("MODE-METRIC g2diag-event clip=\(clip) capsule=\(event.capsule)"
                         + " kind=\(event.kind) t=\(event.seriesIndex)"
                         + " warmed_frame=\(event.warmedFrame)"
                         + " capsule_mode=\(Self.code(event.before))>\(Self.code(event.after))"
                         + " rule3=\(event.rule3Before ? 1 : 0)>\(event.rule3After ? 1 : 0)"
                         + String(format: " resid_z_max=%.4f resid_z_max_before=%.4f"
                                  + " resid_z_rms=%.4f",
                                  event.residualZMax, event.residualZMaxBefore, event.residualZRMS)
                         + " heads=\(event.heads.count)"
                         + " breakers=[\(breakers)] steady=[\(others)]")
        }
        return lines.joined(separator: "\n")
    }
}

/// DIAGNOSTIC side table, written by `buildTraversal` and read by exactly one
/// `print`. It is deliberately NOT a field on `ClipTraversal`: that type is
/// declared in `MuscleLengthModeTests.swift`, and a census reachable from the
/// object every gate already holds is a census one refactor away from being
/// read by a gate. File-private so nothing outside this file can even name it.
private var muscleModeFlipCensusByClip: [String: MuscleModeFlipCensus] = [:]

/// Guards `muscleModeFlipCensusByClip` on BOTH faces. `buildTraversal` writes it
/// under `MuscleLengthModeTests.traversalLock`, but that lock is `private` to
/// `MuscleLengthModeTests.swift` and cannot be named here, so the read at
/// `flipCensusReport` was unguarded — inconsistent with the discipline
/// `traversalCache` establishes next door (adversarial review, 2026-08-21).
/// Benign in a serial XCTest process; a diagnostic that is sloppier than the
/// thing it observes is still sloppy.
private let muscleModeFlipCensusLock = NSLock()

extension MuscleLengthModeTests {

    /// The next-step-51 census for a clip as ONE multi-line greppable string.
    ///
    /// Populated as a side effect of `buildTraversal`, which the G2 methods have
    /// already called (through the cached `traversal(clip:context:)`) before they
    /// print this. If the traversal was never built — or threw before the pass
    /// that records — this says so instead of returning an empty string that
    /// would read as "no flips".
    static func flipCensusReport(clip: String) -> String {
        muscleModeFlipCensusLock.lock()
        let stored = muscleModeFlipCensusByClip[clip]
        muscleModeFlipCensusLock.unlock()
        guard let census = stored else {
            return "MODE-METRIC g2diag clip=\(clip) census=ABSENT"
                + " reason=buildTraversal_did_not_run_for_this_clip_in_this_process"
                + " note=absence_is_not_zero_flips"
        }
        return census.report()
    }
}
