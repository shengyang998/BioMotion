import XCTest
@testable import BioMotion

/// WHAT THIS FILE IS FOR
/// ─────────────────────────────────────────────────────────────────────────
/// `MuscleActivationResult.relativeTorqueResidual` is the only signal that
/// separates "the muscles reproduced the inverse-dynamics torques" from "the
/// ‖a‖² regularizer won and the torques were abandoned" — the τ match is a soft
/// penalty, not a constraint. For that signal to mean anything, the vector it
/// norms has to be homogeneous.
///
/// Two things could make it inhomogeneous, and this file measures both rather
/// than assuming either:
///
///   1. UNITS. `FullBody.osim` coordinates are not all rotational. A coordinate
///      driven by a `<TransformAxis name="translationN">` is a displacement in
///      METRES, so its generalised force is a FORCE IN NEWTONS and its
///      "moment arm" −∂L_MT/∂q is DIMENSIONLESS. Summing those squares with
///      newton-metre squares is a unit error.
///      `testTranslationalCoordinateCensus` counts them from the XML.
///
///   2. COORDINATES NO MUSCLE CAN ACTUATE. If no muscle crosses a coordinate,
///      every activation vector produces exactly zero generalised force there,
///      so its row contributes ‖τ_j‖ to the residual no matter what the solver
///      does. Those rows are unreachable by construction and only inflate the
///      number. `testMomentArmCensus` measures how many there are and, just as
///      importantly, whether the moment-arm magnitudes separate cleanly enough
///      that the cut is a structural fact rather than a tuned constant.
///
/// Everything here preserves the former calculation sequence
/// (IK → Savitzky-Golay → unvalidated diagnostic ID+GRF →
/// `computeMomentArms` → `solveReal`),
/// so its algebra remains characterisable without exposing the unconstrained
/// ID result to product code.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHAT THE MEASUREMENTS SAID (2026-08-07). Two of the three starting
/// hypotheses were WRONG, and the wrong ones were wrong in the flattering
/// direction.
/// ─────────────────────────────────────────────────────────────────────────
///
/// 1. "The QP minimises over all 169 coordinates." NO. It already restricted
///    to the 159 with a non-zero moment-arm column. The 10 it drops carry a
///    combined demand of 0.0001 / 0.03 / 0.20 Nm on the three poses — they
///    were never in the residual, so there was nothing to win here.
///
/// 2. "FullBody.osim has 6 sternum + 72 rib coordinates whose generalised
///    force is a force in newtons." NO — six in the whole model, and only
///    three of them (SternumX/Y/Z) are in the QP. Every `T*_r*_{X,Y,Z}` rib
///    coordinate is a `rotation1/2/3` axis in spite of its name.
///
/// 3. "SternumY's 72.5 N is inflating the residual." BACKWARDS. It is matched
///    to 0.02 N and sat in the DENOMINATOR, flattering neutral standing by
///    2.1× (0.1245 mixed-unit vs 0.2662 on the moment rows). It is also not
///    junk: 72.697 N = 9.81 × 7.4104 kg is exactly the shoulder girdle plus
///    both arms, whose only load path is the sternocostal joint.
///
/// What the residual actually is, measured three ways:
///   * INVARIANT TO WEIGHT — sweeping λ over 1 → 1e8 moves it ≤ 11%, upward.
///     So it is a reachability distance, not an objective-weighting artifact.
///   * NOT A CAPACITY SHORTFALL — 0 of 159 rows demand more than
///     Σ|R|·F_max·f_AL·f_FV·cosα at that pose.
///   * IN STANDING IT IS THE ACTIVATION FLOOR — A_eff·(a_min·1) puts 10.89 Nm
///     on the 72 costovertebral rows (demand 1.68 Nm) and the solve can only
///     add to it. Achieved rib residual: 9.89 Nm, i.e. 87% of the total.
///     Dropping just those rows takes standing to 0.0163.
///
/// Historical PELVIS-root result, unit-consistent and locked rows excluded
/// (measured 2026-08-07; the 2026-08-06 values are in brackets):
///   neutral standing 0.2008 [0.2008] · 4° lean 0.1526 [0.1526] · dancer
///   0.3545 [0.6406].
///
/// THE DANCER MOVED AND NOTHING IN THIS FILE'S SUBJECT CHANGED. `MuscleSolver`
/// and `MomentArmComputer` are untouched between those two columns; the IK
/// stage became a fixed point. The 2026-08-06 note said the dancer's residual
/// was a report on a pose that was wrong before the muscle solver saw it — it
/// used to miss its own markers by 3.4–3.7 cm RMS and land on two different
/// poses (‖q‖ 4.52 vs 5.06) depending only on how many poses preceded it. It
/// then solved reproducibly at ‖q‖ 5.746453 and 2.1224 cm true marker RMS, and
/// the residual almost halved.
///
/// CURRENT SOURCE CONTRACT (2026-08-10): the dancer uses MHR_ROOT rather than
/// mislabelling raw MHR joint 1 as PELVIS. This deliberately unscaled diagnostic
/// measures 1.53645 cm RMS; the production source-aware scaling path measures
/// 1.2758 cm. The unscaled diagnostic's shipped relative torque residual moves
/// to 0.5939547. Better marker fit therefore did not make the dancer a clean
/// muscle benchmark; standing remains the stage's controlled benchmark.
final class MuscleQPUnitsTests: XCTestCase {

    private var bridge: NimbleBridge!
    private var computer: MomentArmComputer!

    override func setUp() {
        super.setUp()
        continueAfterFailure = true
    }

    // MARK: - Model loading
    //
    // POSE ISOLATION. Two pieces of state leak between poses, and both change
    // the answer:
    //   * `NimbleBridge` carries a rolling ground-height estimate and an IK warm
    //     start, so a pose solved after a taller one can be told it is not
    //     touching the floor — which silently switches the legacy diagnostic to
    //     zero-external-force inverse dynamics. `resetSessionState()` clears
    //     both, which is what `perPose()` below is for.
    //   * `MuscleSolver` warm-starts OSQP from the previous activations and
    //     reuses the factorized workspace, so at its 1e-3 tolerance the returned
    //     activations depend on solve ORDER. Each pose therefore gets a fresh
    //     solver.
    // Reusing both made the dancer read `relative = 0.730 / saturated = 3` in an
    // earlier revision of this file versus `0.612 / 17` cold.
    //
    // The model is loaded ONCE. `NimbleBridge.mm` shares one skeleton across
    // instances, and re-loading it while an earlier `MomentArmComputer` still
    // points at the old one crashed the test runner.

    private func osimPath() throws -> String {
        try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
    }

    private func loadFullBodyOnce() throws {
        guard bridge == nil else { return }
        let path = try osimPath()
        let b = NimbleBridge()
        XCTAssertTrue(b.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
        let c = MomentArmComputer()
        XCTAssertTrue(c.parseMusclePaths(fromOsimPath: path, from: b))
        bridge = b
        computer = c
    }

    /// Fresh per-pose state: cold ground-height estimator, cold IK warm start,
    /// cold OSQP workspace.
    private func perPose() throws -> MuscleSolver {
        try loadFullBodyOnce()
        bridge.resetSessionState()
        let solver = MuscleSolver()
        XCTAssertTrue(solver.loadMuscles(fromOsimPath: try osimPath()))
        return solver
    }

    // MARK: - Poses
    //
    // Two independent poses, because a single pose cannot distinguish "this
    // coordinate has no musculature" (a model fact, pose-invariant) from "this
    // muscle happens to have a near-zero moment arm here" (a pose fact).
    //
    // The standing marker set is the same construction as
    // `StaticEquilibriumBenchmarkTests` — adult segment lengths that agree with
    // FullBody.osim's own neutral geometry to <= 0.5 mm, trunk vertical, arms
    // down, feet flat. It is duplicated rather than shared because that file is
    // owned by a different change; if the two ever disagree the benchmark's
    // copy is authoritative.

    private enum Anthro {
        static let ankleHeight   = 0.0750
        static let shankLength   = 0.4000
        static let thighLength   = 0.4060
        static let pelvisRise    = 0.0785
        static let hipX          = -0.0563
        static let kneeX         = -0.0527
        static let ankleX        = -0.0627
        static let ankleToToeX   =  0.1300
        static let ankleToToeY   = -0.0440
        static let hipHalfWidth   = 0.0773
        static let kneeHalfWidth  = 0.0770
        static let ankleHalfWidth = 0.0770
        static let toeHalfWidth   = 0.0860
        static var pelvisY: Double { ankleHeight + shankLength + thighLength + pelvisRise }
        static var hipY: Double   { ankleHeight + shankLength + thighLength }
        static var kneeY: Double  { ankleHeight + shankLength }
        static var toeY: Double   { ankleHeight + ankleToToeY }
        static var toeX: Double   { ankleX + ankleToToeX }
    }

    private static let trunkOffsets: [(String, Double, Double, Double)] = [
        ("SPINE_L", -0.1019,  0.1447,  0.0000),
        ("SPINE_M", -0.1595,  0.3849,  0.0000),
        ("C7",      -0.1173,  0.5217,  0.0000),
        ("NECK",    -0.1130,  0.5426,  0.0000),
        ("HEAD",    -0.1130,  0.6926,  0.0000),
        ("RSJC",    -0.0997,  0.4715,  0.1706),
        ("LSJC",    -0.0997,  0.4715, -0.1706),
        ("REJC",    -0.0865,  0.1853,  0.1610),
        ("LEJC",    -0.0865,  0.1853, -0.1610),
        ("RWJC",    -0.1020, -0.0636,  0.2007),
        ("LWJC",    -0.1020, -0.0636, -0.2007),
    ]

    private static func standingMarkers(leanRad: Double) -> [(String, SIMD3<Double>)] {
        let ax = Anthro.ankleX
        let ay = Anthro.ankleHeight
        func lean(_ p: SIMD3<Double>) -> SIMD3<Double> {
            let dx = p.x - ax, dy = p.y - ay
            let c = cos(leanRad), s = sin(leanRad)
            return SIMD3<Double>(ax + dx * c - dy * s, ay + dx * s + dy * c, p.z)
        }
        var out: [(String, SIMD3<Double>)] = []
        for (name, sign) in [("RTOE", 1.0), ("LTOE", -1.0)] {
            out.append((name, SIMD3<Double>(Anthro.toeX, Anthro.toeY, sign * Anthro.toeHalfWidth)))
        }
        for (name, sign) in [("RAJC", 1.0), ("LAJC", -1.0)] {
            out.append((name, SIMD3<Double>(ax, ay, sign * Anthro.ankleHalfWidth)))
        }
        for (name, sign) in [("RKJC", 1.0), ("LKJC", -1.0)] {
            out.append((name, lean(SIMD3<Double>(Anthro.kneeX, Anthro.kneeY, sign * Anthro.kneeHalfWidth))))
        }
        for (name, sign) in [("RHJC", 1.0), ("LHJC", -1.0)] {
            out.append((name, lean(SIMD3<Double>(Anthro.hipX, Anthro.hipY, sign * Anthro.hipHalfWidth))))
        }
        let pelvis = SIMD3<Double>(0.0, Anthro.pelvisY, 0.0)
        out.append(("PELVIS", lean(pelvis)))
        for (name, dx, dy, dz) in Self.trunkOffsets {
            out.append((name, lean(SIMD3<Double>(pelvis.x + dx, pelvis.y + dy, pelvis.z + dz))))
        }
        return out
    }

    private static var dancerMarkers: [(String, SIMD3<Double>)] {
        OfflineMuscleChainFixture.markers.map {
            ($0.1, SIMD3<Double>(Double($0.2.x), Double($0.2.y), Double($0.2.z)))
        }
    }

    // MARK: - Shared plumbing (historical photo-chain diagnostic)

    private struct Solved {
        let dofNames: [String]
        let q: [NSNumber]
        let dq: [NSNumber]
        let torques: [NSNumber]
        var tau: [Double] { torques.map { $0.doubleValue } }
    }

    private func solve(markers: [(String, SIMD3<Double>)], tag: String) throws -> Solved {
        var positions: [NSNumber] = []
        var names: [String] = []
        for (opensimName, p) in markers {
            names.append(opensimName)
            positions.append(NSNumber(value: p.x))
            positions.append(NSNumber(value: p.y))
            positions.append(NSNumber(value: p.z))
        }
        let ik = try XCTUnwrap(bridge.solveIK(withMarkerPositions: positions, markerNames: names),
                               "solveIK returned nil")
        let n = ik.jointAngles.count
        var filters = (0..<n).map { _ in SavitzkyGolayFilter() }
        var q: [Double] = [], dq: [Double] = [], ddq: [Double] = []
        for push in 0..<SavitzkyGolayFilter.windowSize {
            q.removeAll(); dq.removeAll(); ddq.removeAll()
            for i in 0..<n {
                if let out = filters[i].push(ik.jointAngles[i].doubleValue, timestamp: Double(push) * 0.5) {
                    q.append(out.pos); dq.append(out.vel); ddq.append(out.acc)
                }
            }
        }
        XCTAssertEqual(q.count, n, "[\(tag)] Savitzky-Golay did not warm up")
        // IK FINGERPRINT. `NimbleIKResult.error` is nimble's LOSS (sum of
        // squared weighted marker residuals), not an RMS. Printed with ‖q‖
        // because the dancer's residual turned out to depend on how many poses
        // were solved before it in the same test method — the IK, not the QP,
        // is what is not reproducible there.
        print("QP-IK [\(tag)] ik_loss=\(ik.error) " +
              "per_marker_rms_m=\((ik.error / Double(names.count)).squareRoot()) " +
              "q_norm=\(q.reduce(0) { $0 + $1 * $1 }.squareRoot())")
        let id = try XCTUnwrap(
            bridge.solveUnvalidatedIDGRFForDiagnostics(
                withJointAngles: q.map { NSNumber(value: $0) },
                jointVelocities: dq.map { NSNumber(value: $0) },
                jointAccelerations: ddq.map { NSNumber(value: $0) }),
            "[\(tag)] unvalidated diagnostic ID+GRF solve returned nil")
        return Solved(dofNames: ik.dofNames,
                      q: q.map { NSNumber(value: $0) },
                      dq: dq.map { NSNumber(value: $0) },
                      torques: id.jointTorques)
    }

    // MARK: - Translational-coordinate census (pure XML, no solver)

    /// A coordinate is TRANSLATIONAL iff every `<TransformAxis>` naming it is a
    /// `translationN` axis. `knee_angle_*` is named by both a `rotation1` axis
    /// (LinearFunction) and by three coupled `translationN` splines, so it is
    /// rotational — its generalised force is a moment.
    static func translationalCoordinates(osimPath: String) -> Set<String> {
        guard let text = try? String(contentsOfFile: osimPath, encoding: .utf8) else { return [] }
        var rotational = Set<String>()
        var translational = Set<String>()
        var axisIsTranslation: Bool?
        for raw in text.split(separator: "\n", omittingEmptySubsequences: false) {
            let line = raw.trimmingCharacters(in: .whitespaces)
            if line.hasPrefix("<TransformAxis name=\"translation") { axisIsTranslation = true; continue }
            if line.hasPrefix("<TransformAxis name=\"rotation") { axisIsTranslation = false; continue }
            if line.hasPrefix("</TransformAxis>") { axisIsTranslation = nil; continue }
            guard let isTranslation = axisIsTranslation,
                  line.hasPrefix("<coordinates>"), line.hasSuffix("</coordinates>") else { continue }
            let inner = line.dropFirst("<coordinates>".count).dropLast("</coordinates>".count)
            for token in inner.split(separator: " ") {
                if isTranslation { translational.insert(String(token)) }
                else { rotational.insert(String(token)) }
            }
        }
        return translational.subtracting(rotational)
    }

    func testTranslationalCoordinateCensus() throws {
        try loadFullBodyOnce()
        let translational = Self.translationalCoordinates(osimPath: try osimPath())
        let dofNames = bridge.dofNames
        let present = dofNames.filter { translational.contains($0) }.sorted()
        print("QP-UNITS coords_total=\(dofNames.count)")
        print("QP-UNITS translational_in_xml=\(translational.count) \(translational.sorted().joined(separator: ","))")
        print("QP-UNITS translational_present_in_skeleton=\(present.count) \(present.joined(separator: ","))")

        // Pinned so a model edit that adds or removes a translational
        // coordinate cannot slip past unnoticed. These six are the only
        // coordinates in FullBody.osim whose generalised force is in newtons:
        // the three ground_pelvis translations and the three at the
        // rib1_R -> sternum CustomJoint. Every T*_r*_X/Y/Z rib coordinate is a
        // rotation1/2/3 axis despite the X/Y/Z naming.
        XCTAssertEqual(present, ["SternumX", "SternumY", "SternumZ",
                                 "pelvis_tx", "pelvis_ty", "pelvis_tz"])
    }

    // MARK: - Moment-arm census

    private struct Census {
        let dofNames: [String]
        let maxAbsR: [Double]        // max_m |R[m, j]|
        let capacity: [Double]       // sum_m |R[m, j]| * F_max,m
        let crossing: [Int]          // muscles with |R| > 1e-6
    }

    private func census(arms: [Double], fMax: [Double], nMuscles: Int, dofNames: [String]) -> Census {
        let n = dofNames.count
        var maxAbsR = [Double](repeating: 0, count: n)
        var capacity = [Double](repeating: 0, count: n)
        var crossing = [Int](repeating: 0, count: n)
        for m in 0..<nMuscles {
            for j in 0..<n {
                let r = abs(arms[m * n + j])
                if r > maxAbsR[j] { maxAbsR[j] = r }
                capacity[j] += r * fMax[m]
                if r > 1e-6 { crossing[j] += 1 }
            }
        }
        return Census(dofNames: dofNames, maxAbsR: maxAbsR, capacity: capacity, crossing: crossing)
    }

    /// THE MEASUREMENT THAT DECIDES WHETHER "HAS A MUSCLE" IS A STRUCTURAL FACT
    /// OR A TUNED CONSTANT.
    ///
    /// The moment arms come from a central difference with a 1e-4 rad stencil on
    /// path lengths of order 0.1-1 m, so forward-kinematics round-off alone
    /// produces |r| of order 1e-11 m. If the per-coordinate maxima were spread
    /// smoothly across that scale, any cut would be a tuning knob. This prints
    /// the distribution so the claim can be checked rather than asserted.
    func testMomentArmCensus() throws {
        let translational = Self.translationalCoordinates(osimPath: try osimPath())

        for (tag, markers) in [("upright", Self.standingMarkers(leanRad: 0.0)),
                               ("lean4deg", Self.standingMarkers(leanRad: -0.070027)),
                               ("dancer", Self.dancerMarkers)] {
            _ = try perPose()
            let fMax = computer.maxIsometricForces.map { $0.doubleValue }
            let nMuscles = computer.muscleNames.count
            let s = try solve(markers: markers, tag: tag)
            let arms = try XCTUnwrap(
                computer.computeMomentArms(withJointAngles: s.q, dofNames: s.dofNames)).map { $0.doubleValue }
            let c = census(arms: arms, fMax: fMax, nMuscles: nMuscles, dofNames: s.dofNames)
            let n = s.dofNames.count

            // Distribution of max_m |R[m, j]| by decade.
            var decades: [Int: Int] = [:]
            var exactZero = 0
            for v in c.maxAbsR {
                if v == 0 { exactZero += 1; continue }
                decades[Int(floor(log10(v))), default: 0] += 1
            }
            print("QP-UNITS [\(tag)] n_coords=\(n) muscles=\(nMuscles) exact_zero_column=\(exactZero)")
            for d in decades.keys.sorted() {
                print("QP-UNITS [\(tag)] maxAbsR_decade 1e\(d) count=\(decades[d]!)")
            }
            for thr in [1e-10, 1e-8, 1e-6, 1e-4, 1e-3] {
                let cnt = c.maxAbsR.filter { $0 > thr }.count
                print("QP-UNITS [\(tag)] coords_with_maxAbsR_gt_\(thr)=\(cnt)")
            }

            // The excluded set, split the way the product question needs it.
            let excluded = (0..<n).filter { c.maxAbsR[$0] <= 1e-6 }
            let excludedTranslational = excluded.filter { translational.contains(s.dofNames[$0]) }
            let excludedRotational = excluded.filter { !translational.contains(s.dofNames[$0]) }
            print("QP-UNITS [\(tag)] excluded_total=\(excluded.count) " +
                  "translational=\(excludedTranslational.count) rotational_unmuscled=\(excludedRotational.count)")
            print("QP-UNITS [\(tag)] excluded_names=\(excluded.map { s.dofNames[$0] }.joined(separator: ","))")
            let includedTranslational = (0..<n).filter {
                c.maxAbsR[$0] > 1e-6 && translational.contains(s.dofNames[$0])
            }
            for j in includedTranslational {
                print("QP-UNITS [\(tag)] INCLUDED_TRANSLATIONAL \(s.dofNames[j]) " +
                      "maxAbsR=\(c.maxAbsR[j]) crossing=\(c.crossing[j]) " +
                      "capacity_N=\(c.capacity[j]) tau_N=\(s.tau[j])")
            }

            // Demand vs capacity on the rows that survive, ranked by how much
            // of the residual they can possibly account for.
            let ranked = (0..<n).sorted { abs(s.tau[$0]) > abs(s.tau[$1]) }.prefix(12)
            for j in ranked {
                print("QP-UNITS [\(tag)] top_demand \(s.dofNames[j]) tau=\(s.tau[j]) " +
                      "capacity=\(c.capacity[j]) maxAbsR=\(c.maxAbsR[j]) " +
                      "translational=\(translational.contains(s.dofNames[j]))")
            }
        }
    }

    // MARK: - Where the residual actually lives

    /// Attributes the residual of the CURRENT solver's own solution across three
    /// groups of rows: coordinates no muscle crosses, translational coordinates
    /// (newtons), and ordinary rotational coordinates (newton-metres). Nothing
    /// here changes the solver; it only decomposes the number it already
    /// reports, so the decomposition is a measurement of the shipped behaviour.
    func testResidualAttributionByRowGroup() throws {
        let translational = Self.translationalCoordinates(osimPath: try osimPath())

        for (tag, markers) in [("upright", Self.standingMarkers(leanRad: 0.0)),
                               ("lean4deg", Self.standingMarkers(leanRad: -0.070027)),
                               ("dancer", Self.dancerMarkers)] {
            let solver = try perPose()
            let fMax = computer.maxIsometricForces.map { $0.doubleValue }
            let nMuscles = computer.muscleNames.count
            let s = try solve(markers: markers, tag: tag)
            let n = s.dofNames.count
            let armsNS = try XCTUnwrap(
                computer.computeMomentArms(withJointAngles: s.q, dofNames: s.dofNames))
            let arms = armsNS.map { $0.doubleValue }
            let c = census(arms: arms, fMax: fMax, nMuscles: nMuscles, dofNames: s.dofNames)

            let result = try XCTUnwrap(
                solver.solveReal(withJointTorques: s.torques,
                                 momentArms: armsNS,
                                 muscleNames: computer.muscleNames,
                                 muscleLengths: computer.currentMuscleLengths,
                                 maxForces: computer.maxIsometricForces,
                                 optimalFiberLengths: computer.optimalFiberLengths,
                                 tendonSlackLengths: computer.tendonSlackLengths,
                                 pennationAngles: computer.pennationAngles,
                                 jointVelocities: s.dq,
                                 dofNames: s.dofNames,
                                 dt: 0.5,
                                 softPenalty: 100.0))
            let a = result.activations.map { $0.doubleValue }
            let atFloor = a.filter { $0 <= solver.minActivation + 1e-6 }.count
            let saturated = a.filter { $0 >= 1.0 - 1e-6 }.count
            print("QP-UNITS [\(tag)] SOLVE residualNm=\(result.torqueResidualNm) " +
                  "relative=\(result.relativeTorqueResidual) saturated=\(saturated) " +
                  "at_floor=\(atFloor) converged=\(result.converged)")

            // Reconstruct the per-row residual from the returned activations.
            // A_eff[j, m] = R[m, j] * forceScale[m]; forceScale is internal, so
            // recover the row product directly from the reported muscle forces
            // (force_m = a_m * forceScale_m), which is exactly what the solver
            // multiplied by R.
            let force = result.forces.map { $0.doubleValue }
            var rowResidual = [Double](repeating: 0, count: n)
            for m in 0..<nMuscles {
                for j in 0..<n { rowResidual[j] += arms[m * n + j] * force[m] }
            }
            for j in 0..<n { rowResidual[j] -= s.tau[j] }

            func norm(_ idx: [Int], _ v: [Double]) -> Double {
                idx.reduce(0.0) { $0 + v[$1] * v[$1] }.squareRoot()
            }
            let unmuscled = (0..<n).filter { c.maxAbsR[$0] <= 1e-6 }
            let transRows = (0..<n).filter { c.maxAbsR[$0] > 1e-6 && translational.contains(s.dofNames[$0]) }
            let rotRows = (0..<n).filter { c.maxAbsR[$0] > 1e-6 && !translational.contains(s.dofNames[$0]) }
            // Costovertebral coordinates, named T<n>_r<n>{L,R}_{X,Y,Z}. All 72
            // are ROTATIONS despite the X/Y/Z suffix.
            let isRib: (String) -> Bool = { $0.range(of: "^T[0-9]+_r[0-9]+[LR]_[XYZ]$",
                                                     options: .regularExpression) != nil }
            let ribRows = rotRows.filter { isRib(s.dofNames[$0]) }
            let nonRibRotRows = rotRows.filter { !isRib(s.dofNames[$0]) }

            print("QP-UNITS [\(tag)] rows unmuscled=\(unmuscled.count) " +
                  "translational_muscled=\(transRows.count) rotational_muscled=\(rotRows.count) " +
                  "of_which_rib=\(ribRows.count)")
            print("QP-UNITS [\(tag)] residual_norm unmuscled=\(norm(unmuscled, rowResidual)) " +
                  "translational=\(norm(transRows, rowResidual)) rotational=\(norm(rotRows, rowResidual)) " +
                  "rib=\(norm(ribRows, rowResidual)) nonrib_rot=\(norm(nonRibRotRows, rowResidual))")
            print("QP-UNITS [\(tag)] tau_norm unmuscled=\(norm(unmuscled, s.tau)) " +
                  "translational=\(norm(transRows, s.tau)) rotational=\(norm(rotRows, s.tau)) " +
                  "rib=\(norm(ribRows, s.tau)) nonrib_rot=\(norm(nonRibRotRows, s.tau))")
            let rotOnlyRelative = norm(rotRows, rowResidual) / max(norm(rotRows, s.tau), 1e-6)
            print("QP-UNITS [\(tag)] rotational_only_relative=\(rotOnlyRelative)")
            let nonRibRelative = norm(nonRibRotRows, rowResidual) / max(norm(nonRibRotRows, s.tau), 1e-6)
            print("QP-UNITS [\(tag)] nonrib_rotational_relative=\(nonRibRelative)")
            // How much of the demand each group carries, and how much of it the
            // muscles reproduce, in the group's own units.
            for (gname, idx) in [("rib", ribRows), ("nonrib_rot", nonRibRotRows),
                                 ("translational", transRows)] {
                let maxTau = idx.map { abs(s.tau[$0]) }.max() ?? 0
                let maxRes = idx.map { abs(rowResidual[$0]) }.max() ?? 0
                print("QP-UNITS [\(tag)] group \(gname) max|tau|=\(maxTau) max|residual|=\(maxRes)")
            }

            // STATE capacity, not structural capacity. The QP's matrix is
            // R * diag(F_max * f_AL * f_FV * cos a), so the torque a coordinate
            // can actually receive at THIS pose is sum_m |R| * forceScale_m, not
            // sum_m |R| * F_max_m. The two differ by however much the
            // force-length curve has collapsed, and only the first one bounds
            // what the solver can do. Recovered from the reported forces:
            // forceScale_m = force_m / a_m.
            var stateCapacity = [Double](repeating: 0, count: n)
            var forceScale = [Double](repeating: 0, count: nMuscles)
            for m in 0..<nMuscles where a[m] > 0 { forceScale[m] = force[m] / a[m] }
            for m in 0..<nMuscles {
                for j in 0..<n { stateCapacity[j] += abs(arms[m * n + j]) * forceScale[m] }
            }
            let deadMuscles = forceScale.filter { $0 < 1e-9 }.count
            let fMaxSum = fMax.reduce(0, +)
            print("QP-UNITS [\(tag)] forceScale zero_muscles=\(deadMuscles) of \(nMuscles) " +
                  "sum_forceScale=\(forceScale.reduce(0, +)) sum_Fmax=\(fMaxSum)")
            let overState = (0..<n).filter { abs(s.tau[$0]) > stateCapacity[$0] && c.maxAbsR[$0] > 1e-6 }
            print("QP-UNITS [\(tag)] rows_over_STATE_capacity=\(overState.count) of \(rotRows.count + transRows.count)")
            for j in overState.sorted(by: { abs(s.tau[$0]) - stateCapacity[$0] > abs(s.tau[$1]) - stateCapacity[$1] }).prefix(8) {
                print("QP-UNITS [\(tag)] over_state \(s.dofNames[j]) tau=\(s.tau[j]) " +
                      "state_capacity=\(stateCapacity[j]) struct_capacity=\(c.capacity[j])")
            }

            // WHAT THE ACTIVATION FLOOR COSTS BEFORE THE OPTIMIZER CHOOSES
            // ANYTHING. Every activation is bounded below by aMin = 0.02, so
            // A_eff * (aMin * 1) is a moment field the solve cannot remove, only
            // add to. If that field is comparable to tau, the floor — a value
            // STATUS.md records as having been tuned so the visualisation would
            // not go "permanently blue" — is itself a large part of the residual.
            let aMin = solver.minActivation
            var floorMoment = [Double](repeating: 0, count: n)
            for m in 0..<nMuscles {
                let f = aMin * forceScale[m]
                for j in 0..<n { floorMoment[j] += arms[m * n + j] * f }
            }
            print("QP-UNITS [\(tag)] floor_field_norm rib=\(norm(ribRows, floorMoment)) " +
                  "nonrib_rot=\(norm(nonRibRotRows, floorMoment)) " +
                  "translational=\(norm(transRows, floorMoment))")
            print("QP-UNITS [\(tag)] floor_field_vs_tau nonrib_rot=" +
                  "\(norm(nonRibRotRows, floorMoment) / max(norm(nonRibRotRows, s.tau), 1e-9))")

            let worst = (0..<n).sorted { abs(rowResidual[$0]) > abs(rowResidual[$1]) }.prefix(10)
            for j in worst {
                print("QP-UNITS [\(tag)] worst_row \(s.dofNames[j]) residual=\(rowResidual[j]) " +
                      "tau=\(s.tau[j]) struct_capacity=\(c.capacity[j]) state_capacity=\(stateCapacity[j]) " +
                      "translational=\(translational.contains(s.dofNames[j]))")
            }
        }
    }

    // MARK: - Is the leftover residual a WEIGHT, a ROW SET, or the PHYSICS?
    //
    // Three candidate mechanisms, each with a measurement that can rule it out:
    //
    //   WEIGHT.  The torque match is a soft penalty with lambda = 100 against an
    //            epsA = 0.01 activation regularizer. If lambda is simply too
    //            small, raising it collapses the residual. If the residual
    //            plateaus, the target is not reachable and no weight can help.
    //
    //   ROW SET. Removing a row is done by zeroing that coordinate's COLUMN of
    //            the moment-arm matrix, which drops it from `activeDOFs` inside
    //            the solver without touching any other row. Three candidate cuts:
    //            the model's own <locked>true</locked> coordinates, all 72
    //            costovertebral coordinates, and the 3 translational ones.
    //
    //   PHYSICS. 520 unknowns, ~159 equations, box bounds [aMin, 1]. The
    //            reachable set is a zonotope; tau may just be outside it.
    //
    // Reported so the three are attributable independently.

    private struct QPRun {
        let residual: Double
        let relative: Double
        let forceResidual: Double
        let relativeForce: Double
        let saturated: Int
        let atFloor: Int
        var line: String {
            "residualNm=\(residual) relative=\(relative) " +
            "forceResidualN=\(forceResidual) relativeForce=\(relativeForce) " +
            "saturated=\(saturated) at_floor=\(atFloor)"
        }
    }

    private func runQP(_ solver: MuscleSolver,
                       arms: [NSNumber],
                       s: Solved,
                       lambda: Double) throws -> QPRun {
        let r = try XCTUnwrap(
            solver.solveReal(withJointTorques: s.torques,
                             momentArms: arms,
                             muscleNames: computer.muscleNames,
                             muscleLengths: computer.currentMuscleLengths,
                             maxForces: computer.maxIsometricForces,
                             optimalFiberLengths: computer.optimalFiberLengths,
                             tendonSlackLengths: computer.tendonSlackLengths,
                             pennationAngles: computer.pennationAngles,
                             jointVelocities: s.dq,
                             dofNames: s.dofNames,
                             dt: 0.5,
                             softPenalty: lambda))
        let a = r.activations.map { $0.doubleValue }
        return QPRun(residual: r.torqueResidualNm,
                     relative: r.relativeTorqueResidual,
                     forceResidual: r.forceResidualN,
                     relativeForce: r.relativeForceResidual,
                     saturated: a.filter { $0 >= 1.0 - 1e-6 }.count,
                     atFloor: a.filter { $0 <= solver.minActivation + 1e-6 }.count)
    }

    /// Zeroes the moment-arm COLUMNS of the named coordinates, which is exactly
    /// how a row leaves the QP: `MuscleSolver` keeps a coordinate only if some
    /// muscle has a non-zero moment arm about it.
    private func armsDropping(_ drop: Set<String>, arms: [NSNumber], dofNames: [String],
                              nMuscles: Int) -> [NSNumber] {
        let n = dofNames.count
        var out = arms
        let idx = (0..<n).filter { drop.contains(dofNames[$0]) }
        for m in 0..<nMuscles {
            for j in idx { out[m * n + j] = NSNumber(value: 0.0) }
        }
        return out
    }

    func testResidualMechanismSweep() throws {
        let translational = Self.translationalCoordinates(osimPath: try osimPath())
        let locked = Set(FullBodyDOFFixture.xmlLockedCoordinates)
        let isRib: (String) -> Bool = { $0.range(of: "^T[0-9]+_r[0-9]+[LR]_[XYZ]$",
                                                 options: .regularExpression) != nil }

        for (tag, markers) in [("upright", Self.standingMarkers(leanRad: 0.0)),
                               ("dancer", Self.dancerMarkers)] {
            var solver = try perPose()
            let s = try solve(markers: markers, tag: tag)
            let armsNS = try XCTUnwrap(
                computer.computeMomentArms(withJointAngles: s.q, dofNames: s.dofNames))
            let nMuscles = computer.muscleNames.count

            // --- WEIGHT ---------------------------------------------------
            // THE DECISIVE ONE. If the residual is flat in lambda, the soft
            // penalty is already behaving as a hard least-squares projection
            // and what is left is the distance from tau to the reachable set.
            // **WHICH lambdas may be read is decided by a THEOREM about the
            // objective, not by whether the answer is convenient.**
            //
            // `MuscleSolver` minimises `½ε‖a‖² + ½λ‖Aa − τ‖²` over the box
            // `a ∈ [aMin, 1]` (the shipped form drops the `−½λ‖τ‖²` constant).
            // For `λ₁ < λ₂` with minimisers `a₁`, `a₂` and residuals
            // `rᵢ = ‖Aaᵢ − τ‖²`, optimality of each at its own λ gives
            //     ½ε‖a₁‖² + ½λ₁r₁ ≤ ½ε‖a₂‖² + ½λ₁r₂
            //     ½ε‖a₂‖² + ½λ₂r₂ ≤ ½ε‖a₁‖² + ½λ₂r₁
            // and adding them collapses to `½(λ₂ − λ₁)(r₂ − r₁) ≤ 0`, i.e.
            // **r₂ ≤ r₁**. The argument uses only convexity of the feasible set,
            // so the BOX does not weaken it. AT THE MINIMISER THE τ-RESIDUAL IS
            // NON-INCREASING IN λ. A returned point whose residual went UP did
            // not minimise, and its residual is not a measurement of the
            // objective — the same class of error as the all-at-floor corner
            // below, and as "a gate that measured nothing is not a gate that
            // passed".
            //
            // This replaces the ad-hoc rule that admitted every solve except the
            // all-at-floor corner and excused one known-bad point in a comment
            // ("it happens once, at lambda = 100 on the `dancer` pose"). That
            // point is no longer degenerate — `scaling = 0` + `polishing = 1`
            // solves it to relative 0.3407 — and the degeneracy moved to
            // λ ≥ 1e6, where `εI + λAᵀA` is conditioned 1e13–1e15. Measured
            // 2026-08-09: upright 0.2420 / 0.2373 / 0.2336 for λ ≤ 1e4 and then
            // **0.5833** at 1e6 (a 149.6 % rise) and 1.4760 at 1e8 (531.8 %);
            // dancer 0.3392 / 0.3407 / 0.3390 and then **0.9535** at both
            // (181.3 %). The 1 % slack below is what a converged solve at this
            // tolerance may legitimately wobble by — dancer's λ=100 rises 0.44 %
            // and is admitted — and it is three orders below the violations.
            var byLambda: [Double] = []
            var admittedLambdas: [Double] = []
            var rejected: [String] = []
            var best = Double.infinity
            let lambdas = [1.0, 10.0, 100.0, 1e3, 1e4, 1e5, 1e6, 1e8]
            let shippingLambda = 100.0
            for lambda in lambdas {
                solver = try perPose()
                solver.excludesLockedCoordinates = false
                let r = try runQP(solver, arms: armsNS, s: s, lambda: lambda)
                // The all-at-floor corner: `relativeForce >= 1` says its forces
                // explain none of the demand. Kept as its own reason so the log
                // still names it; it is now also caught by the monotone test.
                let allAtFloor = r.atFloor == nMuscles && r.relativeForce >= 1.0
                let breaksMonotonePath = r.relative > best * 1.01
                if allAtFloor || breaksMonotonePath {
                    rejected.append(String(format: "lambda=%g relative=%.6f best_so_far=%.6f "
                                           + "rise=%.1f%% all_at_floor=%@",
                                           lambda, r.relative, best,
                                           100 * (r.relative / best - 1),
                                           allAtFloor ? "yes" : "no"))
                } else {
                    byLambda.append(r.relative)
                    admittedLambdas.append(lambda)
                    best = Swift.min(best, r.relative)
                }
                print("QP-SWEEP [\(tag)] lambda=\(lambda) \(r.line)")
            }
            print("QP-SWEEP [\(tag)] admitted_lambdas=\(admittedLambdas) "
                  + "rejected_not_minimisers=\(rejected)")

            // A rejection is only readable as "the solver ran out of decades" if
            // the rejected set is the TOP of the sweep. An interior rejection
            // would mean the exclusion rule is picking points, and the spread
            // below would be over a set chosen after seeing the answer.
            let admittedSuffix = lambdas.prefix(admittedLambdas.count)
            XCTAssertEqual(Array(admittedSuffix), admittedLambdas,
                "\(tag): the lambdas that failed the monotone test are not a contiguous high-end "
                + "tail (\(rejected)) — the exclusion is selecting points, not bounding the range "
                + "over which this solver returns minimisers")
            // THE SHIPPING WEIGHT MUST BE ONE OF THEM. `MuscleSolver`'s
            // `softPenalty` is 100 on every real frame, so a sweep that admits
            // four decades none of which is the shipped one says nothing about
            // the product. This assertion did not exist before and would have
            // failed on the 2026-08-08 `dancer` case the old comment excused.
            XCTAssertTrue(admittedLambdas.contains(shippingLambda),
                "\(tag): the shipping soft penalty \(shippingLambda) did not return a minimiser "
                + "(\(rejected)), so every activation the app produces at this pose is a point "
                + "the objective does not choose")
            XCTAssertGreaterThanOrEqual(byLambda.count, 4,
                "\(tag): fewer than four usable solves, so there is no sweep to read")
            let spread = (byLambda.max()! - byLambda.min()!) / max(byLambda.min()!, 1e-9)
            let decades = log10(admittedLambdas.last! / admittedLambdas.first!)
            print("QP-SWEEP [\(tag)] admitted_relative_spread=\(spread) decades=\(decades)")
            // THE FALSIFIER, stated as the mechanism claim rather than as a
            // tolerance. "The residual is a reachability distance, not an
            // artifact of the objective weighting" is false if buying more
            // decades of tau-match weight materially BUYS something. Over the
            // decades where this solver returns minimisers it does not, and the
            // number got BETTER when the exclusion stopped comparing minimisers
            // with non-minimisers: measured 2026-08-09, upright spans
            // 0.2336-0.2420 (spread 0.0356) and dancer 0.3390-0.3407 (0.0050),
            // against 0.229 / 0.029 on 2026-08-08 and a bar of 0.5.
            //
            // What the sweep no longer claims: the range is the decades the
            // shipping solver can actually solve, not eight. Nothing here can
            // say what the residual would be at λ = 1e8, because nothing in this
            // repo can compute that point — `scaling = 0` lands 7.7e-02 from the
            // minimiser there and `scaling = 10` landed 0.46-0.65, and neither
            // moves with eps down to 1e-7 or 50,000 iterations.
            XCTAssertLessThan(spread, 0.5,
                "\(tag): buying \(decades) decades of tau-match weight moved the relative residual "
                + "by \(spread) of itself (\(byLambda) at \(admittedLambdas)). That would mean the "
                + "leftover residual IS the objective weighting after all, and the reachability "
                + "reading in MuscleSolver.h is wrong.")

            // --- ROW SET --------------------------------------------------
            let cuts: [(String, Set<String>)] = [
                ("none", []),
                ("locked", locked),
                ("ribs", Set(s.dofNames.filter(isRib))),
                ("translational", translational),
                ("ribs+translational", Set(s.dofNames.filter(isRib)).union(translational)),
            ]
            for (name, drop) in cuts {
                solver = try perPose()
                solver.excludesLockedCoordinates = false
                let cut = armsDropping(drop, arms: armsNS, dofNames: s.dofNames, nMuscles: nMuscles)
                let r = try runQP(solver, arms: cut, s: s, lambda: 100.0)
                print("QP-SWEEP [\(tag)] cut=\(name) dropped=\(drop.intersection(s.dofNames).count) \(r.line)")
            }
        }
    }

    // MARK: - The shipped configuration, before and after

    /// BEFORE  = every coordinate with a non-zero moment-arm column, one mixed
    ///           newton / newton-metre norm. This is what shipped up to
    ///           2026-08-07.
    /// AFTER   = the same minus the model's `<locked>true</locked>`
    ///           coordinates, with the moment rows and the force rows reported
    ///           as separate norms.
    ///
    /// The two changes are applied one at a time so their effects are
    /// attributable, and the numbers are printed rather than only asserted,
    /// because the honest headline is that ONE of them makes the reported
    /// number worse.
    func testShippedConfigurationBeforeAndAfter() throws {
        for (tag, markers) in [("upright", Self.standingMarkers(leanRad: 0.0)),
                               ("lean4deg", Self.standingMarkers(leanRad: -0.070027)),
                               ("dancer", Self.dancerMarkers)] {
            var solver = try perPose()
            let s = try solve(markers: markers, tag: tag)
            let arms = try XCTUnwrap(
                computer.computeMomentArms(withJointAngles: s.q, dofNames: s.dofNames))

            // REPRODUCIBILITY PROBE, and it is not decoration. Two runs of what
            // should have been the identical configuration disagreed by 8% on
            // the dancer across two test methods, so before any before/after
            // claim is made the same configuration is solved three times from
            // three cold solvers here. Input fingerprints are printed alongside
            // so a disagreement can be attributed to the inputs or to OSQP.
            let lengths = computer.currentMuscleLengths.map { $0.doubleValue }
            let armsD = arms.map { $0.doubleValue }
            print("QP-FINAL [\(tag)] inputs sum_abs_arms=\(armsD.reduce(0) { $0 + abs($1) }) " +
                  "sum_lengths=\(lengths.reduce(0, +)) " +
                  "tau_norm=\(s.tau.reduce(0) { $0 + $1 * $1 }.squareRoot())")
            for rep in 0..<3 {
                solver = try perPose()
                solver.excludesLockedCoordinates = false
                let r = try runQP(solver, arms: arms, s: s, lambda: 100.0)
                print("QP-FINAL [\(tag)] repeat\(rep) \(r.line)")
            }

            solver = try perPose()
            solver.excludesLockedCoordinates = false
            let before = try runQP(solver, arms: arms, s: s, lambda: 100.0)
            print("QP-FINAL [\(tag)] A_unlocked_split \(before.line)")

            solver = try perPose()
            solver.excludesLockedCoordinates = true
            let after = try runQP(solver, arms: arms, s: s, lambda: 100.0)
            print("QP-FINAL [\(tag)] B_shipped \(after.line)")

            // Recombine the two norms the way the old code did, so the
            // pre-change number is reproducible from this test alone.
            let mixedBefore = (before.residual * before.residual
                               + before.forceResidual * before.forceResidual).squareRoot()
            print("QP-FINAL [\(tag)] mixed_unit_equivalent_of_A=\(mixedBefore)")
        }
    }

    /// Pins the two model facts the row rule rests on, so a model edit that
    /// invalidates them fails here instead of silently changing the residual.
    func testSolverAgreesWithAnIndependentReadOfTheModel() throws {
        let solver = try perPose()
        let xmlTranslational = Self.translationalCoordinates(osimPath: try osimPath())
        XCTAssertEqual(Set(solver.translationalCoordinateNames), xmlTranslational,
                       "MuscleSolver's translational-coordinate parse disagrees with this " +
                       "file's independent scan of the same XML")
        XCTAssertEqual(Set(solver.lockedCoordinateNames),
                       Set(FullBodyDOFFixture.xmlLockedCoordinates),
                       "MuscleSolver's <locked> parse disagrees with the generated fixture")
        XCTAssertEqual(solver.lockedCoordinateNames.count, 54)
        XCTAssertEqual(solver.translationalCoordinateNames.count, 6)
    }
}
