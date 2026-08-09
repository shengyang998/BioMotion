import XCTest
@testable import BioMotion

/// Historical straight-line baseline plus the current no-wrap control.
///
/// This measurement motivated path wrapping before both surface solvers shipped.
/// It remains the control that checks paths with no wrap object against OpenSim.
///
/// Two questions, and they are NOT the same question:
///
/// 1. Is "OpenSim with every `WrapObject` deactivated" a faithful stand-in for
///    what this code computes? If it is, the wrap-off column isolates the
///    missing wrap solver from every other difference between two independent
///    implementations (Nimble's FK vs Simbody's, live `MovingPathPoint` and
///    `ConditionalPathPoint` evaluation, a 1e-4 rad centred difference vs
///    OpenSim's `MomentArmSolver`).
/// 2. How far is the shipped number from the reference?
///
/// The suite answers 1 first, because a large answer to 1 would mean the
/// wrap-off column is measuring something else and question 2 would have to be
/// asked differently.
final class StraightLinePathErrorTests: XCTestCase {

    /// Poses this suite evaluates. Every one costs a full 169-coordinate
    /// central-difference sweep over all 520 muscles, so this is a subset of the
    /// 173 in the fixture, chosen to include the named poses plus a stride
    /// through the sweeps and the grid.
    private static let namedPoses = ["neutral", "squat_deep", "spine_flexed"]
    private static let poseStride = 6

    private struct Sample {
        let pose: String
        let muscle: String
        let coordinate: String
        let ours: Double
        let wrapOff: Double
        let wrapOn: Double
    }

    private static var samples: [Sample] = []
    /// Declared by the fixture generator, never inferred from the numbers.
    private static var carriesPathWrap: Set<String> = []
    private static var setupFailure: String?
    private static var dofNameMismatch: (missing: [String], extra: [String])?

    override func setUpWithError() throws {
        try Self.build(bundle: Bundle(for: type(of: self)))
        if let failure = Self.setupFailure { throw RequiredTestDependencyError(failure) }
    }

    private static func build(bundle: Bundle) throws {
        guard samples.isEmpty, setupFailure == nil else { return }
        guard let path = bundle.path(forResource: "FullBody", ofType: "osim") else {
            setupFailure = "FullBody.osim is not reachable from the test bundle"
            return
        }
        let table = try OpenSimReferenceFixture.load(bundle: bundle)

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

        // Feeding a name the skeleton does not carry is silently ignored by
        // `computeMomentArms`, so the pose would be partly the fixture's and
        // partly whatever the skeleton was last left at. Check the two name
        // sets agree before trusting a single number below.
        let ourNames = Set(bridge.dofNames as [String])
        let fixtureNames = Set(table.coordinateNames)
        dofNameMismatch = (missing: fixtureNames.subtracting(ourNames).sorted(),
                           extra: ourNames.subtracting(fixtureNames).sorted())

        let muscleIndex = Dictionary(uniqueKeysWithValues:
            (computer.muscleNames as [String]).enumerated().map { ($0.element, $0.offset) })
        let coordinateColumn = Dictionary(uniqueKeysWithValues:
            table.coordinateNames.enumerated().map { ($0.element, $0.offset) })

        var poseIndices = Set<Int>()
        for (index, pose) in table.poses.enumerated()
        where namedPoses.contains(pose.id) || index % poseStride == 0 {
            poseIndices.insert(index)
        }
        for (index, pose) in table.poses.enumerated() where pose.id.hasPrefix("run_") {
            poseIndices.insert(index)
        }

        carriesPathWrap = Set(table.muscles.filter(\.carriesPathWrap).map(\.name))

        let dofNames = table.coordinateNames
        var collected: [Sample] = []
        for poseIndex in poseIndices.sorted() {
            let pose = table.poses[poseIndex]
            let angles = pose.values.map { NSNumber(value: $0) }
            guard let flat = computer.computeMomentArms(withJointAngles: angles,
                                                        dofNames: dofNames) else {
                setupFailure = "computeMomentArms returned nil at pose \(pose.id)"
                return
            }
            let columns = dofNames.count
            for (fixtureMuscle, muscle) in table.muscles.enumerated() {
                guard let ourRow = muscleIndex[muscle.name],
                      let row = table.row(pose: poseIndex, muscle: fixtureMuscle) else { continue }
                for (slot, coordinate) in muscle.coordinates.enumerated() {
                    guard let column = coordinateColumn[coordinate] else { continue }
                    collected.append(Sample(pose: pose.id,
                                            muscle: muscle.name,
                                            coordinate: coordinate,
                                            ours: flat[ourRow * columns + column].doubleValue,
                                            wrapOff: row.momentArmsWrapOff[slot],
                                            wrapOn: row.momentArmsWrapOn[slot]))
                }
            }
        }
        samples = collected
    }

    // MARK: - Question 0: are we even talking about the same coordinates

    func testNimbleAndOpenSimAgreeOnTheCoordinateSet() throws {
        let mismatch = try XCTUnwrap(Self.dofNameMismatch)
        XCTAssertEqual(mismatch.missing, [],
                       "coordinates the fixture names that nimble's skeleton does not carry")
        XCTAssertEqual(mismatch.extra, [],
                       "DOFs nimble carries that the fixture does not name")
    }

    func testSamplesWereActuallyCollected() {
        XCTAssertGreaterThan(Self.samples.count, 1000,
                             "nothing was measured, so every number below is vacuous")
    }

    // MARK: - Question 1: is wrap-off a faithful stand-in for our code

    /// **The tripwire FIRED on 2026-08-08, which is what it was for.**
    ///
    /// It used to assert that `MomentArmComputer` reproduces OpenSim with
    /// wrapping DISABLED — true exactly as long as the wrap solver was missing.
    /// Cylinder wrapping ships now, so the muscles that carry a `PathWrap` no
    /// longer track that column, and the assertion moved to
    /// `CylinderWrapValidationTests` where it belongs (against the wrap-ON
    /// reference and OpenSim's own derivative of its own length).
    ///
    /// What remains here is the half of the claim that is still true and still
    /// load-bearing: on the muscles with NO wrap object — where there is nothing
    /// to solve — the two independent straight-line implementations must remain
    /// inside the fixed 5 mm cross-implementation contract. This is a control
    /// and tripwire, not a runtime-generated inclusion threshold. FullBody's
    /// four MovingPathPoints are parsed and none is approximated.
    func testOurStraightLineTracksOpenSimOnMusclesWithNoWrapObject() {
        let unwrapped = Self.samples.filter { !Self.carriesPathWrap.contains($0.muscle) }
        let wrapped = Self.samples.filter { Self.carriesPathWrap.contains($0.muscle) }
        let differences = unwrapped.map { abs($0.ours - $0.wrapOff) }
        print(Self.describe(differences, label: "NO-WRAP muscles: ours vs OpenSim wrap-OFF"))
        print(Self.describe(wrapped.map { abs($0.ours - $0.wrapOff) },
                            label: "WRAPPED muscles: ours vs wrap-OFF (must NOT be small now)"))
        XCTAssertGreaterThan(unwrapped.count, 0)
        XCTAssertGreaterThan(wrapped.count, 0)
        let worst = differences.max() ?? .infinity
        XCTAssertLessThan(worst, 0.005,
                          "the shipped straight line and OpenSim's straight line must "
                          + "agree to a few mm where no wrap object exists, or every "
                          + "fixed 5 mm control contract is violated")
        XCTAssertGreaterThan(wrapped.map { abs($0.ours - $0.wrapOff) }.max() ?? 0, 0.05,
                             "the WRAPPED muscles must have left the wrap-OFF column behind "
                             + "by centimetres; if they have not, the wrap solver is not "
                             + "running and CylinderWrapValidationTests is passing vacuously")
    }

    // MARK: - Question 2: how far is the shipped number from the reference

    /// Straight-line-era headline, kept as the before/after record. `ours` is
    /// now the WRAPPED path, so the gap to the reference has collapsed; what
    /// this asserts is that the collapse happened.
    func testShippedMomentArmsAgainstTheOpenSimReference() {
        let toReference = Self.samples.map { abs($0.ours - $0.wrapOn) }
        let wrapShare = Self.samples.map { abs($0.wrapOff - $0.wrapOn) }
        print(Self.describe(toReference, label: "ours vs OpenSim reference (wrap ON)"))
        print(Self.describe(wrapShare, label: "wrap-OFF vs wrap-ON (the wrap solver's share)"))
        print(Self.worstOffenders(by: { abs($0.ours - $0.wrapOn) },
                                  label: "largest ours-vs-REFERENCE errors"))

        let worstToReference = toReference.max() ?? 0
        let worstWrapShare = wrapShare.max() ?? 0
        XCTAssertGreaterThan(worstWrapShare, 0.10,
                             "the wrap solver's own share of the old error was 14.7 cm; if "
                             + "this drops, the reference or the model changed")
        XCTAssertLessThan(worstToReference, 0.02,
                          "with wrapping shipped, no sampled moment arm may be more than "
                          + "2 cm from the reference — it was 14.66 cm before")
    }

    /// No `String(format:"%s")` here: it needs a C string, and every route to
    /// one (`(s as NSString).utf8String`) is an Optional whose force-unwrap
    /// would trap inside the test host. Pad in Swift instead.
    private static func worstOffenders(by metric: (Sample) -> Double,
                                       label: String) -> String {
        let ranked = samples.sorted { metric($0) > metric($1) }.prefix(5)
        var lines = ["\(label):"]
        for sample in ranked {
            let columns = pad(sample.pose, 22) + pad(sample.muscle, 16)
                + pad(sample.coordinate, 20)
            lines.append("    " + columns
                + String(format: "ours %+.5f  off %+.5f  on %+.5f",
                         sample.ours, sample.wrapOff, sample.wrapOn))
        }
        return lines.joined(separator: "\n")
    }

    private static func pad(_ text: String, _ width: Int) -> String {
        text.count >= width ? text + " "
                            : text + String(repeating: " ", count: width - text.count)
    }

    private static func describe(_ values: [Double], label: String) -> String {
        guard !values.isEmpty else { return "\(label): no samples" }
        let sorted = values.sorted()
        func percentile(_ p: Double) -> Double {
            let k = (Double(sorted.count) - 1) * p
            let low = Int(k.rounded(.down))
            let high = min(low + 1, sorted.count - 1)
            return sorted[low] + (sorted[high] - sorted[low]) * (k - Double(low))
        }
        return String(format:
            "%@: n=%d  median %.6f m  p90 %.6f  p99 %.6f  max %.6f",
            label, sorted.count, percentile(0.5), percentile(0.9),
            percentile(0.99), sorted[sorted.count - 1])
    }
}
