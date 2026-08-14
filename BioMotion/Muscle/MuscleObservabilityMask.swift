import Foundation

/// The DERIVED suppression mask for the length-mode layer — never a hand-picked
/// list.
///
/// Registered 2026-08-13 (round-4, adjudicated 2026-08-14) as FIVE rules; any
/// hit renders the capsule in the neutral anatomy colour with a STATED reason.
/// Nothing here may name a muscle: the whole point of a derivation is that a
/// model change moves the mask, and a literal list would silently stop tracking
/// the model. The gate that pins this asserts a literal count of zero
/// displayed-muscle names in this file.
///
/// ## Rule 0 — name resolution and family expansion
///
/// A capsule maps to the UNIQUE model muscle whose OSIM path-point names are
/// prefixed `<capsule name>-`. Verified against the artifact: 520 muscle blocks
/// (422 Thelen2003 + 98 Millard2012), census EXACT 18 / ALIAS 6 / ZERO 2 /
/// MULTI 0 = 26, resolving 24. The six aliases exist because the model names the
/// MUSCLE with a numeric suffix while naming its path points without it, so a
/// bare exact-name lookup on `MuscleDef.name` misses all six. Two capsules
/// resolve to NOTHING — the model's erector-spinae family is named differently
/// and its members are thoracic — and they are suppressed BY THE DERIVATION.
/// **0 or ≥2 matches → SUPPRESS, never guess.**
///
/// FAMILY EXPANSION: a capsule that visually stands for a multi-head family
/// displays that WHOLE family, derived from the model's own naming — a resolved
/// muscle whose name carries a numeric head index expands to every model muscle
/// sharing its stem, index position and side. On this model that expands exactly
/// two capsule pairs to three heads each and leaves every other capsule as its
/// own single muscle. Displayed model muscle set = 24 + 8 = 32, feeding 24
/// coloured capsules.
///
/// ## Rule 1 — DRIVE-AWARE IDENTIFIABILITY
///
/// This REPLACES a per-column observability-fraction rule that was structurally
/// incapable of seeing this drive's actual failure. Two defects, both fatal
/// here: (a) a rank-deficient null DIRECTION that is a linear COMBINATION of
/// individually-large columns has every column norm large and is invisible to
/// any per-column test; (b) it used the 20-marker set when the analysed clip
/// supplies a different one.
///
/// At a pose `q`, build the marker Jacobian `J(q)` from EXACTLY the markers the
/// analysed clip submits (3 rows per marker, UNWEIGHTED, m/rad). Take the SVD.
/// Let `N(q)` be the span of the right singular vectors whose singular value is
/// `< sigmaVisible`. Then
///
///     nullFraction_j(q) = ‖ P_N(q) e_j ‖₂  ∈ [0,1],   P_N = V_null V_nullᵀ
///
/// Coordinate `j` is IDENTIFIED at `q` iff `nullFraction_j(q) ≤ 0.5`. This
/// measure detects a null direction that is a combination of large columns,
/// which is precisely what a per-column norm cannot.
///
/// `0.5` is NOT a new constant: it is `PostureFindings.depthSuppressionFraction`,
/// whose doc calls it "the equal-contribution crossover, not a tuned constant".
///
/// `sigmaVisible = 1.0e-2 m/rad`, FROZEN, two-sided derivation. The UPPER bound
/// is measured and hard: it must sit strictly below the 0.0343 m/rad
/// marker-Jacobian column norm of the axial-humeral-rotation coordinate at the
/// model's neutral pose, because masking that DOF was MEASURED as a regression
/// (+0.717 cm marker RMS, 1.536 → 2.253; IK convergence 0 → 123 iterations) and
/// rejected; `1.0e-2` is 3.43× below it. The LOWER bound: a direction moving
/// markers by 10 mm/rad produces, over the ±0.5 rad plausible excursion of a
/// lower-limb coordinate, marker motion of order 5 mm — the same order as the
/// model's own best achieved marker-fit residual (1.276 cm source-aware-scaled,
/// 1.536 cm unscaled). A direction whose entire plausible excursion hides inside
/// the fit residual is not measured. **This lower side has NO test pin and that
/// is disclosed.**
///
/// SPANNING: muscle `m` spans coordinate `j` iff `|R[m,j]| ≥ 1.0e-6 m/rad`
/// (1 µm per radian; typical moment arms are ~1e-2 m). SUPPRESSION is ANY, not a
/// fraction: a muscle spanning ANY unidentified coordinate is suppressed.
///
/// CLIP VERDICT is FAIL-CLOSED — coordinate `j` is identified for the clip iff
/// it is identified at EVERY warmed frame. Fail-closed is chosen because the
/// alternative ("identified on ≥ X % of frames") introduces `X`, a post-hoc
/// lever.
///
/// ACCEPTANCE IS INVERTED: "fires on 0" is DISPROOF, not success.
///
/// ## Rule 2 — unmeasured coordinate (clip face only)
///
/// Distinct from Rule 1, because a coordinate can be visible to the drive yet
/// never exercised by the clip. Coordinate `j` is UNMEASURED iff
/// `max_t q_j − min_t q_j ≤ 1.0e-6` rad over the warmed frames — the clip never
/// moved it past the solver's own pinned identical-marker fixed-point bound, so
/// it is pinned at seed and its residual is an artefact of the seed, not a
/// measurement of noise. Deliberately NOT `σ̂ == 0`. On the fixture face `σ̂` is
/// the frozen constant vector, so Rule 2 is INAPPLICABLE there and must not be
/// read as suppressing every muscle on the fixture-face gates.
///
/// ## Rule 3 — per-frame indeterminacy (not a mask, a frame-level abstention)
///
/// INDETERMINATE on any frame where `MomentArmComputer` reports an unresolved
/// wrap-switch sample (`lastUnresolvedDiscontinuitySamples`) or the SG filter is
/// not yet warm. The shipped counter is per-CALL rather than per-muscle, so this
/// implementation abstains for the whole admitted set on such a frame — strictly
/// more conservative than the registered rule, and recorded as such.
///
/// ## Rule 4 — multi-head unanimity
///
/// A capsule standing for a family colours ONLY when every head agrees in mode;
/// otherwise it renders neutral. A RENDER rule, not a scoring rule: the
/// fixture-face gates score per NAMED MODEL MUSCLE, so expanding the display set
/// only enlarges their populations.
enum MuscleObservabilityMask {

    // MARK: Frozen constants

    /// The equal-contribution crossover. Reused, not re-derived, from the
    /// posture layer's depth-suppression threshold.
    static var identifiedNullFractionCeiling: Double { PostureFindings.depthSuppressionFraction }

    /// `sigmaVisible`, metres of marker motion per radian. Frozen; its upper
    /// bound is pinned by test against the measured must-not-mask column norm.
    static let visibleSingularValueMetresPerRadian: Double = 1.0e-2

    /// The measured marker-Jacobian column norm of the coordinate this repo
    /// proved must NOT be masked. `sigmaVisible` must stay strictly below it.
    static let mustNotMaskColumnNormMetresPerRadian: Double = 0.0343

    /// `|R[m,j]| ≥ this` ⇒ muscle `m` spans coordinate `j`. 1 µm per radian.
    static let spanThresholdMetresPerRadian: Double = 1.0e-6

    /// Peak-to-peak below this over the warmed frames ⇒ the clip never moved the
    /// coordinate past the solver's own identical-marker fixed-point bound.
    static let unmeasuredCoordinateRangeRadians: Double = 1.0e-6

    // MARK: Rule 0 — resolution

    enum ResolutionKind: String {
        /// The capsule name IS the model muscle name.
        case exact
        /// The path-point prefix resolves to a differently-named model muscle.
        case alias
        /// No model muscle carries path points with this prefix.
        case zero
        /// More than one does — SUPPRESS, never pick.
        case multi
    }

    struct CapsuleResolution: Equatable {
        let capsule: String
        let kind: ResolutionKind
        /// The whole displayed family. Empty for `.zero` and `.multi`.
        let modelMuscles: [String]
        var isResolved: Bool { kind == .exact || kind == .alias }
    }

    /// Muscle name → its path-point names, scanned out of an `.osim`.
    ///
    /// A deliberately small, dependency-free scan: it walks the text once,
    /// latching the enclosing muscle block's `name=` attribute and collecting
    /// the `name=` of every path-point element inside it. The alternative — a
    /// full XML parse — buys nothing here and drags a second failure mode into a
    /// derivation that has to be auditable by eye.
    static func pathPointNamesByMuscle(osimText: String) -> [String: [String]] {
        var out: [String: [String]] = [:]
        var currentMuscle: String?
        var depthOfMuscle = 0
        var depth = 0

        // Element tags that END a muscle block are recognised by matching the
        // opening tag we latched, so nested <objects> cannot close it early.
        var muscleTag: String?

        var index = osimText.startIndex
        while let open = osimText[index...].firstIndex(of: "<") {
            guard let close = osimText[open...].firstIndex(of: ">") else { break }
            let raw = String(osimText[osimText.index(after: open)..<close])
            index = osimText.index(after: close)
            if raw.hasPrefix("?") || raw.hasPrefix("!") { continue }

            let selfClosing = raw.hasSuffix("/")
            let isClosing = raw.hasPrefix("/")
            let body = isClosing ? String(raw.dropFirst())
                                 : (selfClosing ? String(raw.dropLast()) : raw)
            let tag = String(body.prefix(while: { !$0.isWhitespace }))

            if isClosing {
                depth -= 1
                // A muscle block opened at depth `d` leaves the scanner at
                // `d + 1`, so its own closing tag brings the depth back TO `d`,
                // not below it. `<` here left every muscle block open for the
                // rest of the file and attributed all 520 muscles' path points
                // to the first one — G3(vi) caught it as EXACT 0 / ALIAS 24
                // instead of 18 / 6.
                if let m = muscleTag, tag == m, depth <= depthOfMuscle {
                    currentMuscle = nil
                    muscleTag = nil
                }
                continue
            }

            let name = attributeValue(named: "name", in: body)
            if currentMuscle == nil, tag.hasSuffix("Muscle"), let name, !name.isEmpty {
                currentMuscle = name
                muscleTag = tag
                depthOfMuscle = depth
                if out[name] == nil { out[name] = [] }
            } else if let muscle = currentMuscle, isPathPointTag(tag), let name, !name.isEmpty {
                out[muscle, default: []].append(name)
            }
            if !selfClosing { depth += 1 }
        }
        return out
    }

    private static func isPathPointTag(_ tag: String) -> Bool {
        tag == "PathPoint" || tag == "MovingPathPoint" || tag == "ConditionalPathPoint"
    }

    private static func attributeValue(named attribute: String, in body: String) -> String? {
        guard let range = body.range(of: "\(attribute)=\"") else { return nil }
        let rest = body[range.upperBound...]
        guard let end = rest.firstIndex(of: "\"") else { return nil }
        return String(rest[..<end])
    }

    /// Rule 0, whole. `capsules` comes from the overlay's own anatomical list;
    /// this file names none of them.
    static func resolve(capsules: [String],
                        pathPointNames: [String: [String]]) -> [CapsuleResolution] {
        let modelNames = Set(pathPointNames.keys)
        return capsules.map { capsule in
            let prefix = capsule + "-"
            let hits = pathPointNames
                .filter { $0.value.contains { $0.hasPrefix(prefix) } }
                .keys
                .sorted()
            switch hits.count {
            case 1:
                let resolved = hits[0]
                let kind: ResolutionKind = (resolved == capsule) ? .exact : .alias
                return CapsuleResolution(capsule: capsule,
                                         kind: kind,
                                         modelMuscles: family(of: resolved, inModel: modelNames))
            case 0:
                return CapsuleResolution(capsule: capsule, kind: .zero, modelMuscles: [])
            default:
                return CapsuleResolution(capsule: capsule, kind: .multi, modelMuscles: [])
            }
        }
    }

    /// Family expansion, derived from the model's own naming: `<stem><index><side>`
    /// expands to every model muscle sharing `stem` and `side` with any index. A
    /// name with no numeric index before its side suffix is its own family.
    static func family(of muscle: String, inModel modelNames: Set<String>) -> [String] {
        guard let parts = stemIndexSide(of: muscle) else { return [muscle] }
        let members = modelNames.filter { candidate in
            guard let other = stemIndexSide(of: candidate) else { return false }
            return other.stem == parts.stem && other.side == parts.side
        }
        return members.isEmpty ? [muscle] : members.sorted()
    }

    private static func stemIndexSide(of name: String) -> (stem: String, side: String)? {
        guard name.count > 3 else { return nil }
        let side = String(name.suffix(2))
        guard side == "_r" || side == "_l" else { return nil }
        let head = String(name.dropLast(2))
        let digits = head.reversed().prefix(while: { $0.isNumber })
        guard !digits.isEmpty else { return nil }
        let stem = String(head.dropLast(digits.count))
        guard !stem.isEmpty else { return nil }
        return (stem, side)
    }

    // MARK: Rule 1 — identifiability

    /// `nullFraction_j = ‖P_N e_j‖₂` for every coordinate, from the UNWEIGHTED
    /// marker Jacobian.
    ///
    /// Computed through the SMALL side. `J` is `3·markers × coordinates` with far
    /// more columns than rows, so its row space is at most `rows`-dimensional:
    /// eigendecompose the `rows × rows` Gram matrix `G = J Jᵀ`, take
    /// `σᵢ = √λᵢ`, lift each retained left vector to `vᵢ = Jᵀuᵢ / σᵢ`, and use
    /// `‖P_N eⱼ‖² = 1 − Σ_{σᵢ ≥ σ_visible} vᵢ[j]²`. Exact, and it never forms a
    /// `columns × columns` matrix — which for this model would be 169 × 169 of
    /// squared data, the worst possible conditioning for the small-σ regime the
    /// whole rule lives in.
    ///
    /// A column that is identically zero contributes nothing to any `vᵢ` and
    /// therefore returns exactly 1.0, which is the behaviour the registration's
    /// forced bound rests on.
    static func nullFractions(jacobianRowMajor: [Double],
                              rows: Int,
                              columns: Int,
                              sigmaVisible: Double = visibleSingularValueMetresPerRadian) -> [Double]? {
        guard rows > 0, columns > 0, jacobianRowMajor.count == rows * columns else { return nil }

        var gram = [Double](repeating: 0, count: rows * rows)
        for a in 0..<rows {
            for b in a..<rows {
                var sum = 0.0
                for c in 0..<columns {
                    sum += jacobianRowMajor[a * columns + c] * jacobianRowMajor[b * columns + c]
                }
                gram[a * rows + b] = sum
                gram[b * rows + a] = sum
            }
        }

        guard let (eigenvalues, eigenvectors) = symmetricEigen(gram, size: rows) else { return nil }

        var retained = [Double](repeating: 0, count: columns)
        let sigmaSquaredFloor = sigmaVisible * sigmaVisible
        for i in 0..<rows where eigenvalues[i] >= sigmaSquaredFloor {
            let sigma = eigenvalues[i].squareRoot()
            guard sigma > 0 else { continue }
            // vᵢ = Jᵀuᵢ / σᵢ, accumulated as squares directly.
            for c in 0..<columns {
                var component = 0.0
                for r in 0..<rows {
                    component += jacobianRowMajor[r * columns + c] * eigenvectors[r * rows + i]
                }
                let v = component / sigma
                retained[c] += v * v
            }
        }

        return retained.map { max(0.0, min(1.0, 1.0 - $0)).squareRoot() }
    }

    /// Cyclic Jacobi for a small symmetric matrix. Deterministic, no external
    /// linear algebra, and accurate enough by an enormous margin: the threshold
    /// this feeds is `1e-4` in eigenvalue units against a largest eigenvalue of
    /// order 1, while Jacobi's relative error is at machine precision.
    ///
    /// Returns `(eigenvalues, eigenvectors)` with eigenvectors stored COLUMN `i`
    /// at stride `size`, i.e. `eigenvectors[r * size + i]`.
    static func symmetricEigen(_ matrix: [Double], size n: Int) -> ([Double], [Double])? {
        guard n > 0, matrix.count == n * n else { return nil }
        var a = matrix
        var v = [Double](repeating: 0, count: n * n)
        for i in 0..<n { v[i * n + i] = 1 }

        for _ in 0..<100 {
            var off = 0.0
            for p in 0..<n {
                for q in (p + 1)..<n { off += a[p * n + q] * a[p * n + q] }
            }
            if off <= 1e-30 { break }

            for p in 0..<n {
                for q in (p + 1)..<n {
                    let apq = a[p * n + q]
                    if abs(apq) < 1e-300 { continue }
                    let app = a[p * n + p]
                    let aqq = a[q * n + q]
                    let theta = (aqq - app) / (2 * apq)
                    let t: Double = theta >= 0
                        ? 1 / (theta + (1 + theta * theta).squareRoot())
                        : -1 / (-theta + (1 + theta * theta).squareRoot())
                    let c = 1 / (1 + t * t).squareRoot()
                    let s = t * c
                    for k in 0..<n {
                        let akp = a[k * n + p]
                        let akq = a[k * n + q]
                        a[k * n + p] = c * akp - s * akq
                        a[k * n + q] = s * akp + c * akq
                    }
                    for k in 0..<n {
                        let apk = a[p * n + k]
                        let aqk = a[q * n + k]
                        a[p * n + k] = c * apk - s * aqk
                        a[q * n + k] = s * apk + c * aqk
                    }
                    for k in 0..<n {
                        let vkp = v[k * n + p]
                        let vkq = v[k * n + q]
                        v[k * n + p] = c * vkp - s * vkq
                        v[k * n + q] = s * vkp + c * vkq
                    }
                }
            }
        }
        let eigenvalues = (0..<n).map { a[$0 * n + $0] }
        return (eigenvalues, v)
    }

    /// Rule 1's per-frame verdict for one coordinate.
    static func isIdentified(nullFraction: Double) -> Bool {
        nullFraction <= identifiedNullFractionCeiling
    }

    /// Rule 1's FAIL-CLOSED clip verdict: identified iff identified at EVERY
    /// warmed frame.
    static func clipIdentifiedCoordinates(perFrameNullFractions: [[Double]],
                                          coordinateCount: Int) -> Set<Int> {
        guard !perFrameNullFractions.isEmpty else { return [] }
        var identified = Set(0..<coordinateCount)
        for frame in perFrameNullFractions {
            for j in 0..<min(coordinateCount, frame.count) where !isIdentified(nullFraction: frame[j]) {
                identified.remove(j)
            }
        }
        return identified
    }

    // MARK: Rule 1/2 — spanning and suppression

    /// `|R[m,j]| ≥ spanThreshold`.
    static func spannedCoordinates(momentArmRow: [Double]) -> Set<Int> {
        var out = Set<Int>()
        for (j, r) in momentArmRow.enumerated() where abs(r) >= spanThresholdMetresPerRadian {
            out.insert(j)
        }
        return out
    }

    /// Rule 2. Peak-to-peak over the WARMED frames only.
    static func unmeasuredCoordinates(warmedPoses: [[Double]], coordinateCount: Int) -> Set<Int> {
        guard !warmedPoses.isEmpty else { return Set(0..<coordinateCount) }
        var out = Set<Int>()
        for j in 0..<coordinateCount {
            var lo = Double.infinity
            var hi = -Double.infinity
            for pose in warmedPoses where j < pose.count {
                lo = min(lo, pose[j])
                hi = max(hi, pose[j])
            }
            if hi - lo <= unmeasuredCoordinateRangeRadians { out.insert(j) }
        }
        return out
    }

    /// ANY, not a fraction: a muscle spanning ANY unidentified or unmeasured
    /// coordinate is suppressed.
    static func isSuppressed(spanned: Set<Int>, identified: Set<Int>, unmeasured: Set<Int>) -> Bool {
        for j in spanned where !identified.contains(j) || unmeasured.contains(j) { return true }
        return false
    }

    // MARK: Rule 4 — multi-head unanimity

    /// A capsule standing for a family colours ONLY when every head agrees.
    static func unanimousMode(_ modes: [MuscleLengthMode]) -> MuscleLengthMode {
        guard let first = modes.first else { return .indeterminate }
        for mode in modes where mode != first { return .indeterminate }
        return first
    }
}
