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
                       timestamps: timestamps, frames: frames)
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
        let modeStart = Date()

        for w in 0..<out.warmedCount {
            guard let rows = ctx.momentArms(pose: smoothedQ[w], muscles: muscles,
                                            coordinates: stencilNames) else { continue }
            let rule3 = ctx.computer.lastUnresolvedDiscontinuitySamples > 0
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
        for resolution in admittedResolutions {
            let heads = resolution.modelMuscles.compactMap { modeSeries[$0] }
            guard let first = heads.first else { continue }
            var series: [MuscleLengthMode] = []
            for w in 0..<first.count {
                series.append(MuscleObservabilityMask.unanimousMode(heads.map { $0[w] }))
            }
            capsuleSeries[resolution.capsule] = series
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
                                break
                            }
                        }
                    }
                }
                out.greyDenominator += 1
                if series[t].isDefined != series[t - 1].isDefined { out.greyTransitions += 1 }
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
