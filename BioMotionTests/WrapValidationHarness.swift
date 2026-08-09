import XCTest
@testable import BioMotion

/// ONE sweep of `FullBody.osim` through the shipped moment-arm chain, paired
/// with OpenSim 4.6's reference columns, shared by the cylinder and ellipsoid
/// validation suites.
///
/// It is a separate type for one reason: the sweep is 36 poses × 169
/// coordinates × 520 muscles and costs minutes of Debug simulator time, and
/// both suites have to read the SAME numbers. Two harnesses would be two
/// measurements, and a disagreement between them would be unattributable.
///
/// Everything here is measurement and bookkeeping. The claims — the
/// pre-registered gates and what each one means — live in the suites.
enum WrapValidationHarness {

    /// Same pose subset for both suites: every one costs a full sweep.
    static let namedPoses = ["neutral", "squat_deep", "spine_flexed"]
    static let poseStride = 6

    /// EVERY arm-sweep pose, on top of the stride, because the 8 wrap
    /// ellipsoids are all on the humerus and every muscle that carries one is an
    /// elbow muscle. Without these the arm sits at one fixed configuration in 31
    /// of the 36 strided poses, so a large sample count would be a large number
    /// of near-copies of one arm pose. 29 poses: 16 `elbow_sweep_*` over
    /// 0–150 deg and 13 `shoulder_sweep_*` over 0–115 deg.
    static let armPosePrefixes = ["elbow_sweep_", "shoulder_sweep_"]

    /// Whether this build solves the muscle's wraps. Read off the parser's own
    /// fidelity report, never a hand-written list, so it cannot drift from what
    /// the code does.
    enum WrapClass { case solved, unsolved, none }

    /// WHICH surface the muscle's `<PathWrap>`s name. The two solvers have
    /// different tolerances and wildly different cost, so a comparison against
    /// OpenSim that does not stratify on this is averaging two things.
    enum Surface { case cylinderOnly, carriesEllipsoid, none }

    struct Sample {
        let pose: String
        let muscle: String
        let coordinate: String
        let wrapClass: WrapClass
        let surface: Surface
        /// How many `PathWrap`s the muscle carries. 1 = OpenSim's closed-form
        /// path; >1 = its iterative one.
        let wrapCount: Int
        let ours: Double
        let wrapOff: Double
        let wrapOn: Double
        /// OpenSim's own central difference of its own length, or nil when the
        /// muscle carries no `PathWrap` (that fixture covers only wrapped ones).
        let centralDifference: Double?
    }

    struct LengthSample {
        let pose: String
        let muscle: String
        let wrapClass: WrapClass
        let surface: Surface
        let wrapCount: Int
        let ours: Double
        let wrapOn: Double
        let ourWrapPoints: Int
        let referenceWrapPoints: Int
    }

    /// The per-frame cost A/B, measured in ONE process at ONE pose set: the
    /// same solve with every `WrapEllipsoid` active and then deactivated.
    struct CostAB {
        var poses: [String] = []
        var withEllipsoids: [Double] = []
        var withoutEllipsoids: [Double] = []
    }

    /// The Hill-model constants the muscle QP needs, read from the SAME parse
    /// the moment arms came from. They are here rather than re-parsed by a
    /// consumer because a second `MomentArmComputer` would be a second model
    /// load, and because a leak measurement that changed `F_max` between its
    /// two solves would not be measuring the moment arm.
    struct MuscleParameters {
        let maxForce: Double
        let optimalFiberLength: Double
        let tendonSlackLength: Double
        let pennationAngle: Double
    }

    /// The SAME (pose, muscle, coordinate) solved twice — with the ellipsoids
    /// active and with them deactivated — so "did the ellipsoid solver help"
    /// and "which residuals pre-date it" are answered against THIS code rather
    /// than against OpenSim's own wrap-off column, which also carries every
    /// other difference between the two implementations.
    struct AblationSample {
        let pose: String
        let muscle: String
        let coordinate: String
        let wrapCount: Int
        let withEllipsoids: Double
        let withoutEllipsoids: Double
        let wrapOff: Double
        let wrapOn: Double
        let centralDifference: Double?
    }

    private(set) static var samples: [Sample] = []
    private(set) static var lengthSamples: [LengthSample] = []
    private(set) static var setupFailure: String?
    private(set) static var solvedMuscles: Set<String> = []
    private(set) static var unsolvedMuscles: Set<String> = []
    private(set) static var ellipsoidMuscles: Set<String> = []
    private(set) static var fullBodyReport: MusclePathFidelityReport?
    private(set) static var solveMilliseconds: [Double] = []
    private(set) static var discontinuityCounters: (centred: Int, oneSided: Int, unresolved: Int) = (0, 0, 0)
    private(set) static var ellipsoidNumericalRefusals: Int = 0
    private(set) static var costAB = CostAB()
    private(set) static var ablation: [AblationSample] = []
    private(set) static var muscleParameters: [String: MuscleParameters] = [:]

    /// Poses the A/B runs at, on top of the accuracy sweep. Six, not 60: each
    /// one costs TWO full solves. Three whole-body poses and three arm
    /// configurations, because the muscles the ablation is about are elbow
    /// muscles and a pose that does not move the arm cannot separate them.
    private static let costPoses = ["neutral", "squat_deep", "run_1_midstance",
                                    "elbow_sweep_030", "elbow_sweep_090",
                                    "shoulder_sweep_060"]

    /// Builds once per process. Throws `XCTSkip` material as a stored string so
    /// a failure to load the model is not reported as every gate failing.
    static func build(bundle: Bundle) throws {
        guard samples.isEmpty, setupFailure == nil else { return }
        guard let path = bundle.path(forResource: "FullBody", ofType: "osim") else {
            setupFailure = "FullBody.osim is not reachable from the test bundle"
            return
        }
        let table = try OpenSimReferenceFixture.load(bundle: bundle)
        let finiteDifference = try OpenSimFiniteDifferenceFixture.load(bundle: bundle)

        let bridge = NimbleBridge()
        guard bridge.loadModel(fromPath: path) else {
            setupFailure = "NimbleBridge could not load FullBody.osim"
            return
        }
        let computer = MomentArmComputer()
        guard computer.parseMusclePaths(fromOsimPath: path, from: bridge) else {
            setupFailure = "MomentArmComputer could not parse FullBody.osim"
            return
        }
        fullBodyReport = computer.fidelityReport
        unsolvedMuscles = Set(computer.fidelityReport.musclesWithUnmodelledPathWraps)

        let muscleIndex = Dictionary(uniqueKeysWithValues:
            (computer.muscleNames as [String]).enumerated().map { ($0.element, $0.offset) })

        let fmax = computer.maxIsometricForces as [NSNumber]
        let lopt = computer.optimalFiberLengths as [NSNumber]
        let lts = computer.tendonSlackLengths as [NSNumber]
        let penn = computer.pennationAngles as [NSNumber]
        var parameters: [String: MuscleParameters] = [:]
        for (name, row) in muscleIndex where row < fmax.count && row < lopt.count
            && row < lts.count && row < penn.count {
            parameters[name] = MuscleParameters(maxForce: fmax[row].doubleValue,
                                                optimalFiberLength: lopt[row].doubleValue,
                                                tendonSlackLength: lts[row].doubleValue,
                                                pennationAngle: penn[row].doubleValue)
        }
        muscleParameters = parameters
        let coordinateColumn = Dictionary(uniqueKeysWithValues:
            table.coordinateNames.enumerated().map { ($0.element, $0.offset) })

        func classify(_ muscle: OpenSimReferenceFixture.Muscle) -> WrapClass {
            guard muscle.carriesPathWrap else { return .none }
            return unsolvedMuscles.contains(muscle.name) ? .unsolved : .solved
        }
        func surface(_ muscle: OpenSimReferenceFixture.Muscle) -> Surface {
            guard muscle.carriesPathWrap else { return .none }
            return computer.ellipsoidPathWrapCount(forMuscleNamed: muscle.name) > 0
                ? .carriesEllipsoid : .cylinderOnly
        }
        solvedMuscles = Set(table.muscles.filter { classify($0) == .solved }.map(\.name))
        ellipsoidMuscles = Set(table.muscles.filter { surface($0) == .carriesEllipsoid }
                                            .map(\.name))

        var poseIndices = Set<Int>()
        for (index, pose) in table.poses.enumerated()
        where namedPoses.contains(pose.id) || index % poseStride == 0 {
            poseIndices.insert(index)
        }
        for (index, pose) in table.poses.enumerated() where pose.id.hasPrefix("run_") {
            poseIndices.insert(index)
        }
        for (index, pose) in table.poses.enumerated()
        where armPosePrefixes.contains(where: { pose.id.hasPrefix($0) }) {
            poseIndices.insert(index)
        }

        let dofNames = table.coordinateNames
        var collected: [Sample] = []
        var collectedLengths: [LengthSample] = []
        var timings: [Double] = []
        var counters = (centred: 0, oneSided: 0, unresolved: 0)
        var refusals = 0

        for poseIndex in poseIndices.sorted() {
            let pose = table.poses[poseIndex]
            let angles = pose.values.map { NSNumber(value: $0) }
            let start = Date()
            guard let flat = computer.computeMomentArms(withJointAngles: angles,
                                                        dofNames: dofNames) else {
                setupFailure = "computeMomentArms returned nil at pose \(pose.id)"
                return
            }
            timings.append(Date().timeIntervalSince(start) * 1000.0)
            counters.centred += computer.lastCentredDifferenceSamples
            counters.oneSided += computer.lastOneSidedDifferenceSamples
            counters.unresolved += computer.lastUnresolvedDiscontinuitySamples
            refusals += computer.lastEllipsoidNumericalRefusals

            // `computeMomentArms` restores the pose it was given, so these two
            // describe the same configuration the matrix above was taken at.
            let lengths = computer.currentMuscleLengths as [NSNumber]
            let wrapPoints = computer.currentWrapPointCounts as [NSNumber]

            let columns = dofNames.count
            // The two fixtures name their poses the same way, so the row lookup
            // is by ID rather than by a shared index nothing enforces.
            let fdPose = finiteDifference.poseIndex(pose.id)
            for (fixtureMuscle, muscle) in table.muscles.enumerated() {
                guard let ourRow = muscleIndex[muscle.name],
                      let row = table.row(pose: poseIndex, muscle: fixtureMuscle) else { continue }
                let wrapClass = classify(muscle)
                let wrapSurface = surface(muscle)
                let wrapCount = computer.pathWrapCount(forMuscleNamed: muscle.name)
                var fdByCoordinate: [String: Double] = [:]
                if let fdPose, let fdMuscle = finiteDifference.muscleIndex(muscle.name),
                   let fdRow = finiteDifference.row(pose: fdPose, muscle: fdMuscle) {
                    let names = finiteDifference.muscles[fdMuscle].coordinates
                    for (slot, name) in names.enumerated() where slot < fdRow.momentArms.count {
                        fdByCoordinate[name] = fdRow.momentArms[slot]
                    }
                }
                for (slot, coordinate) in muscle.coordinates.enumerated() {
                    guard let column = coordinateColumn[coordinate] else { continue }
                    collected.append(Sample(pose: pose.id,
                                            muscle: muscle.name,
                                            coordinate: coordinate,
                                            wrapClass: wrapClass,
                                            surface: wrapSurface,
                                            wrapCount: wrapCount,
                                            ours: flat[ourRow * columns + column].doubleValue,
                                            wrapOff: row.momentArmsWrapOff[slot],
                                            wrapOn: row.momentArmsWrapOn[slot],
                                            centralDifference: fdByCoordinate[coordinate]))
                }
                guard ourRow < lengths.count, ourRow < wrapPoints.count else { continue }
                collectedLengths.append(LengthSample(pose: pose.id,
                                                     muscle: muscle.name,
                                                     wrapClass: wrapClass,
                                                     surface: wrapSurface,
                                                     wrapCount: wrapCount,
                                                     ours: lengths[ourRow].doubleValue,
                                                     wrapOn: row.lengthWrapOn,
                                                     ourWrapPoints: wrapPoints[ourRow].intValue,
                                                     referenceWrapPoints: row.wrapPoints))
            }
        }
        samples = collected
        lengthSamples = collectedLengths
        solveMilliseconds = timings
        discontinuityCounters = counters
        ellipsoidNumericalRefusals = refusals

        // ---- the paired A/B: same process, same poses, ellipsoids on then off ----
        var ab = CostAB()
        var ablationSamples: [AblationSample] = []
        let columns = dofNames.count
        for poseId in costPoses {
            guard let poseIndex = table.poses.firstIndex(where: { $0.id == poseId }) else { continue }
            let angles = table.poses[poseIndex].values.map { NSNumber(value: $0) }

            _ = computer.setEllipsoidWrapObjectsActive(true)
            var start = Date()
            let withEllipsoids = computer.computeMomentArms(withJointAngles: angles,
                                                           dofNames: dofNames)
            let on = Date().timeIntervalSince(start) * 1000.0

            let deactivated = computer.setEllipsoidWrapObjectsActive(false)
            start = Date()
            let withoutEllipsoids = computer.computeMomentArms(withJointAngles: angles,
                                                              dofNames: dofNames)
            let off = Date().timeIntervalSince(start) * 1000.0
            _ = computer.setEllipsoidWrapObjectsActive(true)

            guard deactivated > 0 else {
                setupFailure = "setEllipsoidWrapObjectsActive changed nothing, so the "
                             + "A/B is the same configuration measured twice"
                return
            }
            ab.poses.append(poseId)
            ab.withEllipsoids.append(on)
            ab.withoutEllipsoids.append(off)

            guard let onMatrix = withEllipsoids, let offMatrix = withoutEllipsoids else {
                setupFailure = "computeMomentArms returned nil during the A/B at \(poseId)"
                return
            }
            let fdPose = finiteDifference.poseIndex(poseId)
            for (fixtureMuscle, muscle) in table.muscles.enumerated()
            where ellipsoidMuscles.contains(muscle.name) {
                guard let ourRow = muscleIndex[muscle.name],
                      let row = table.row(pose: poseIndex, muscle: fixtureMuscle) else { continue }
                var fdByCoordinate: [String: Double] = [:]
                if let fdPose, let fdMuscle = finiteDifference.muscleIndex(muscle.name),
                   let fdRow = finiteDifference.row(pose: fdPose, muscle: fdMuscle) {
                    let names = finiteDifference.muscles[fdMuscle].coordinates
                    for (slot, name) in names.enumerated() where slot < fdRow.momentArms.count {
                        fdByCoordinate[name] = fdRow.momentArms[slot]
                    }
                }
                for (slot, coordinate) in muscle.coordinates.enumerated() {
                    guard let column = coordinateColumn[coordinate] else { continue }
                    ablationSamples.append(AblationSample(
                        pose: poseId,
                        muscle: muscle.name,
                        coordinate: coordinate,
                        wrapCount: computer.pathWrapCount(forMuscleNamed: muscle.name),
                        withEllipsoids: onMatrix[ourRow * columns + column].doubleValue,
                        withoutEllipsoids: offMatrix[ourRow * columns + column].doubleValue,
                        wrapOff: row.momentArmsWrapOff[slot],
                        wrapOn: row.momentArmsWrapOn[slot],
                        centralDifference: fdByCoordinate[coordinate]))
                }
            }
        }
        costAB = ab
        ablation = ablationSamples
    }

    // MARK: - Reporting helpers

    static func percentile(_ values: [Double], _ p: Double) -> Double {
        guard !values.isEmpty else { return .nan }
        let sorted = values.sorted()
        let k = (Double(sorted.count) - 1) * p
        let low = Int(k.rounded(.down))
        let high = min(low + 1, sorted.count - 1)
        return sorted[low] + (sorted[high] - sorted[low]) * (k - Double(low))
    }

    static func describe(_ values: [Double], label: String) -> String {
        guard !values.isEmpty else { return "\(label): no samples" }
        return String(format: "%@: n=%d  median %.6f m  p90 %.6f  p99 %.6f  max %.6f",
                      label, values.count, percentile(values, 0.5), percentile(values, 0.9),
                      percentile(values, 0.99), values.max() ?? 0)
    }

    /// **The RELATIVE moment-arm error this build still carries**, as a
    /// fraction of the reference's own value — the quantity a per-muscle scale
    /// error is expressed in, and therefore the one that can be dropped into a
    /// synthetic rig in place of a guessed `×0.6`.
    ///
    /// - Parameter bases: solver base names (no side suffix) to include.
    /// - Parameter poses: pose ids to include; empty means every pose.
    /// - Parameter minimumReferenceMetres: pairs whose reference arm is smaller
    ///   than this are EXCLUDED and counted separately. A 0.2 mm reference arm
    ///   divides a 0.2 mm disagreement into 100 %, and a muscle with no leverage
    ///   at a joint carries no torque there either — the same exclusion the
    ///   2026-08-08 straight-line measurement used, at the same 1 mm.
    /// - Parameter definitionMatched: use OpenSim's own central difference of
    ///   its own length where the fixture has one (it covers only muscles that
    ///   carry a `PathWrap`), falling back to the analytic column. `false` uses
    ///   the analytic column throughout.
    /// - Returns: the relative errors, and how many pairs were excluded.
    static func relativeMomentArmResiduals(bases: Set<String>,
                                           poses: Set<String> = [],
                                           minimumReferenceMetres: Double = 0.001,
                                           definitionMatched: Bool)
        -> (ratios: [Double], excludedBelowMinimum: Int) {
        var ratios: [Double] = []
        var excluded = 0
        for sample in samples {
            guard poses.isEmpty || poses.contains(sample.pose) else { continue }
            guard let split = GaitLoadSummary.split(sample.muscle),
                  bases.contains(split.base) else { continue }
            let reference = definitionMatched ? (sample.centralDifference ?? sample.wrapOn)
                                              : sample.wrapOn
            guard abs(reference) >= minimumReferenceMetres else { excluded += 1; continue }
            ratios.append(abs(sample.ours - reference) / abs(reference))
        }
        return (ratios, excluded)
    }

    static func worstOffenders(in pool: [Sample], by metric: (Sample) -> Double,
                               label: String) -> String {
        let ranked = pool.sorted { metric($0) > metric($1) }.prefix(6)
        var lines = ["\(label):"]
        for sample in ranked {
            lines.append("    " + pad(sample.pose, 22) + pad(sample.muscle, 16)
                + pad(sample.coordinate, 20)
                + String(format: "ours %+.5f  straight %+.5f  ref %+.5f",
                         sample.ours, sample.wrapOff, sample.wrapOn))
        }
        return lines.joined(separator: "\n")
    }

    static func pad(_ text: String, _ width: Int) -> String {
        text.count >= width ? text + " "
                            : text + String(repeating: " ", count: width - text.count)
    }
}
