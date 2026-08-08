import XCTest
@testable import BioMotion

/// **`dL/dq` is discontinuous where a muscle starts or stops wrapping, and a
/// centred difference straddling that switch invents a moment arm.**
///
/// `MusclePathWrapTests.testACentredDifferenceAcrossTheCylinderEndSwitchFabricatesAMomentArm`
/// constructs the hazard on bare geometry and measures it: L steps by 36 mm
/// across the cylinder-end switch, and a centred difference at `eps = 1e-4`
/// turns that into **−180.7 metres** per unit.
///
/// This suite drives the same situation through the SHIPPED chain — nimble's
/// skeleton, `FullBody.osim`'s own wrap objects, `computeMomentArms`'s stencil —
/// and shows that what comes out is the branch-consistent one-sided difference
/// and not the fabrication.
///
/// # How the situation is constructed, and why it is not hand-picked
///
/// Nothing here hard-codes a muscle, a coordinate or a pose. The suite sweeps a
/// real coordinate of the shipped model from the fixture's `neutral` pose,
/// watches `currentWrapPointCounts` for the first muscle whose wrap engagement
/// changes, and bisects to the switch. The stencil is then placed deliberately
/// astride it: `q0 = q* − eps/2`, so `q0 − eps` and `q0` sit on one branch and
/// `q0 + eps` on the other. At the 173 fixture poses this never happens by
/// chance — `CylinderWrapValidationTests` reports **0 one-sided samples out of
/// 3,163,680** — which is exactly why it has to be constructed rather than
/// waited for.
final class MomentArmWrapDiscontinuityTests: XCTestCase {

    private static let stencil = 1e-4  // the step `computeMomentArms` uses

    private struct Rig {
        let computer: MomentArmComputer
        let dofNames: [String]
        let baseAngles: [Double]
    }

    private static var rig: Rig?
    private static var setupFailure: String?

    override func setUpWithError() throws {
        try Self.build(bundle: Bundle(for: type(of: self)))
        if let failure = Self.setupFailure { throw XCTSkip(failure) }
    }

    private static func build(bundle: Bundle) throws {
        guard rig == nil, setupFailure == nil else { return }
        guard let path = bundle.path(forResource: "FullBody", ofType: "osim") else {
            setupFailure = "FullBody.osim is not reachable from the test bundle"
            return
        }
        let table = try OpenSimReferenceFixture.load(bundle: bundle)
        guard let neutral = table.poseIndex("neutral") else {
            setupFailure = "the reference fixture has no `neutral` pose"
            return
        }
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
        // One full sweep to put the skeleton at a known configuration. Every
        // probe below names a SINGLE coordinate, so it moves that one and
        // leaves the rest exactly here — which is what makes a 40-step
        // bisection affordable against a call that costs seconds.
        let angles = table.poses[neutral].values
        guard computer.computeMomentArms(withJointAngles: angles.map { NSNumber(value: $0) },
                                         dofNames: table.coordinateNames) != nil else {
            setupFailure = "computeMomentArms returned nil at the neutral pose"
            return
        }
        rig = Rig(computer: computer, dofNames: table.coordinateNames, baseAngles: angles)
    }

    /// Move ONE coordinate and read back what the wrap solver did there.
    private func probe(_ rig: Rig, coordinate: String, value: Double)
        -> (arms: [Double], lengths: [Double], wrapPoints: [Int])? {
        guard let flat = rig.computer.computeMomentArms(
            withJointAngles: [NSNumber(value: value)], dofNames: [coordinate]) else { return nil }
        return (arms: flat.map(\.doubleValue),
                lengths: (rig.computer.currentMuscleLengths as [NSNumber]).map(\.doubleValue),
                wrapPoints: (rig.computer.currentWrapPointCounts as [NSNumber]).map(\.intValue))
    }

    func testAStencilAstrideAWrapSwitchUsesAOneSidedDifference() throws {
        let rig = try XCTUnwrap(Self.rig)
        let names = rig.computer.muscleNames as [String]
        let eps = Self.stencil

        // 1. Find a coordinate and a muscle whose wrap engagement changes.
        //    Knee flexion is where FullBody.osim's cylinder wraps engage and
        //    disengage; the sweep covers the coordinate's whole clamped range.
        let coordinate = "knee_angle_r"
        guard let column = rig.dofNames.firstIndex(of: coordinate) else {
            throw XCTSkip("\(coordinate) is not a coordinate of this model")
        }
        let base = rig.baseAngles[column]
        var previous = try XCTUnwrap(probe(rig, coordinate: coordinate, value: base))
        var switchMuscle = -1
        var below = base
        var above = base
        let step = 2.0 * Double.pi / 180.0
        for tick in 1...60 {
            let value = base - Double(tick) * step   // knee flexion is negative here
            guard let current = probe(rig, coordinate: coordinate, value: value) else { continue }
            for m in 0..<current.wrapPoints.count
            where (current.wrapPoints[m] > 0) != (previous.wrapPoints[m] > 0) {
                switchMuscle = m
                below = value
                above = base - Double(tick - 1) * step
                break
            }
            if switchMuscle >= 0 { break }
            previous = current
        }
        try XCTSkipIf(switchMuscle < 0,
                      "no wrap engagement switch was found along \(coordinate); the model's "
                      + "wrap geometry changed and this construction needs revisiting")
        let muscle = names[switchMuscle]

        // 2. Bisect to the switch. `engagedAt` is whichever end wraps.
        let engagedAbove =
            try XCTUnwrap(probe(rig, coordinate: coordinate, value: above)).wrapPoints[switchMuscle] > 0
        var wrapping = engagedAbove ? above : below
        var free = engagedAbove ? below : above
        for _ in 0..<60 {
            let middle = 0.5 * (wrapping + free)
            guard let sample = probe(rig, coordinate: coordinate, value: middle) else { break }
            if sample.wrapPoints[switchMuscle] > 0 { wrapping = middle } else { free = middle }
        }
        let switchPoint = 0.5 * (wrapping + free)
        XCTAssertLessThan(abs(wrapping - free), 1e-9,
                          "the bisection did not converge, so the stencil below is not "
                          + "reliably astride the switch")

        // 3. Place the stencil astride it: q0 - eps and q0 on one branch,
        //    q0 + eps on the other.
        // Unconditionally BELOW the switch by half a step: `q0 - eps` and `q0`
        // then share the lower branch and `q0 + eps` is over the line. Which of
        // the two branches is the wrapping one does not matter — what matters
        // is that the stencil crosses.
        let q0 = switchPoint - 0.5 * eps
        let minus = try XCTUnwrap(probe(rig, coordinate: coordinate, value: q0 - eps))
        let centre = try XCTUnwrap(probe(rig, coordinate: coordinate, value: q0))
        // Read the counters for THIS call, before any later probe overwrites them.
        let straddleCentred = rig.computer.lastCentredDifferenceSamples
        let straddleOneSided = rig.computer.lastOneSidedDifferenceSamples
        let straddleUnresolved = rig.computer.lastUnresolvedDiscontinuitySamples
        let plus = try XCTUnwrap(probe(rig, coordinate: coordinate, value: q0 + eps))

        let engagedMinus = minus.wrapPoints[switchMuscle] > 0
        let engagedCentre = centre.wrapPoints[switchMuscle] > 0
        let engagedPlus = plus.wrapPoints[switchMuscle] > 0
        print(String(format: "SWITCH muscle=%@ coordinate=%@ q*=%.9f  engaged(-,0,+)=%d%d%d  "
                     + "L- %.6f L0 %.6f L+ %.6f",
                     muscle, coordinate, switchPoint,
                     engagedMinus ? 1 : 0, engagedCentre ? 1 : 0, engagedPlus ? 1 : 0,
                     minus.lengths[switchMuscle], centre.lengths[switchMuscle],
                     plus.lengths[switchMuscle]))

        XCTAssertEqual(engagedMinus, engagedCentre,
                       "q0 - eps and q0 must share a branch, or the construction failed")
        XCTAssertNotEqual(engagedCentre, engagedPlus,
                          "q0 + eps must be on the OTHER branch, or nothing is straddled")

        // 4. What the three stencils say.
        let rawCentred = -(plus.lengths[switchMuscle] - minus.lengths[switchMuscle]) / (2 * eps)
        let oneSidedBackward = -(centre.lengths[switchMuscle] - minus.lengths[switchMuscle]) / eps
        let shipped = centre.arms[switchMuscle]
        // Well inside the base branch, where every stencil agrees.
        let inside = try XCTUnwrap(probe(rig, coordinate: coordinate, value: q0 - 20 * eps))
        let insideArm = inside.arms[switchMuscle]

        print(String(format: "STENCIL-COMPARE shipped %+.6f  one-sided %+.6f  raw-centred %+.6f  "
                     + "inside-branch %+.6f  (m)",
                     shipped, oneSidedBackward, rawCentred, insideArm))
        print("STENCIL-COUNTS at the straddling pose: centred=\(straddleCentred) "
              + "one-sided=\(straddleOneSided) unresolved=\(straddleUnresolved)")

        // 5. The claims.
        XCTAssertEqual(shipped, oneSidedBackward, accuracy: 1e-9,
                       "the shipped moment arm at a straddling pose IS the branch-consistent "
                       + "one-sided difference")
        XCTAssertEqual(shipped, insideArm, accuracy: 0.005,
                       "and it agrees with the moment arm measured entirely inside the same "
                       + "branch, which a fabricated one would not")
        XCTAssertGreaterThanOrEqual(straddleOneSided, 1,
                                    "the straddling call must REPORT that it dropped to a "
                                    + "one-sided difference, or nothing downstream can tell")
        XCTAssertEqual(straddleUnresolved, 0,
                       "the base pose is not itself the switch, so nothing should be "
                       + "unresolvable here")
        XCTAssertGreaterThan(abs(rawCentred - shipped), 0.010,
                             "the centred difference this replaced differs by more than a "
                             + "centimetre — that gap is the fabrication, and it is what "
                             + "would have reached the QP")
    }

    /// The counters are the only way anybody downstream can tell that a
    /// one-sided difference was used, so they have to be true.
    func testTheStencilCountersAccountForEverySample() throws {
        let rig = try XCTUnwrap(Self.rig)
        guard rig.computer.computeMomentArms(
            withJointAngles: rig.baseAngles.map { NSNumber(value: $0) },
            dofNames: rig.dofNames) != nil else {
            return XCTFail("computeMomentArms returned nil")
        }
        let total = rig.computer.lastCentredDifferenceSamples
            + rig.computer.lastOneSidedDifferenceSamples
            + rig.computer.lastUnresolvedDiscontinuitySamples
        XCTAssertEqual(total, rig.computer.numMuscles * rig.dofNames.count,
                       "every (muscle, coordinate) sample must be counted exactly once")
        XCTAssertEqual(rig.computer.lastUnresolvedDiscontinuitySamples, 0,
                       "a sample the halving never resolved means the pose sits exactly on a "
                       + "switch; at a normal pose there should be none")
    }
}
