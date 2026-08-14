import XCTest
import simd

@testable import BioMotion

/// The PRE-REGISTERED gate battery for the per-muscle muscle-tendon
/// LENGTH-CHANGE MODE layer (registration frozen 2026-08-13, adjudicated
/// 2026-08-14; the full text is the primary doc comment on
/// `MuscleLengthMode`).
///
/// Every bar here was written down before any number was read. A FAIL is a
/// valid deliverable: it means the offline UI does not ship and the measured
/// numbers go into STATUS. Nothing in this file may be weakened to make a gate
/// pass, and every figure is measured and printed as a greppable `MODE-METRIC`
/// line.
///
/// # THE 2026-08-14 CONVERSION — owner-authorised, STATUS next-step 41 (a)
///
/// Seven methods asserted registered bars that the measurement did not meet, so
/// they were RED and `tools/run_tests.sh all` could not run at all. Under an
/// explicit owner authorisation those seven now **record** their outcome
/// instead of demanding it, following this repo's own precedent
/// (`WrappedMomentArmLeakTests.testTheShippedFlagMatchesWhatTheMeasurementSupports`:
/// compute the gates from live measurement, pin the measured values, assert
/// coherence with the shipped decision — green suite, retired claim).
///
/// In each converted method:
///
/// 1. the REGISTERED BAR is unchanged and is still ASSERTED, through
///    `RegisteredBar` — a silent bar edit still breaks the test;
/// 2. the MEASURED OUTCOME is PINNED to the receipt it was read from
///    (`/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log`, `grep MODE-METRIC`,
///    line numbers cited per method), empty populations pinned as exactly zero;
/// 3. the VERDICT is asserted and printed as a greppable `MODE-VERDICT` line:
///    the gate FAILED against its registered bar, the layer is NOT SHIPPED, and
///    reopening requires richer fixtures — a production-grade 20-marker
///    solved-pose clip — plus a fresh adjudication.
///
/// So "no expected number is hardcoded" now holds for the BARS and not for the
/// OUTCOMES. The pins are deliberate, dated and directional in BOTH senses: a
/// number that DRIFTS and a number that IMPROVES past its pin both turn this
/// file red, because either one means the mask, the fixtures, the classifier or
/// the model moved and the verdict has to be re-adjudicated rather than
/// silently re-baselined. Nothing here may be repaired by editing a pin.
///
/// Converted: `testG2NonDegeneracyOnThePinnedClips`,
/// `testG3KneeIsIdentifiedUnderTheFiveMarkerDriveAtFixturePoses`,
/// `testG4PhysiologyDirectionsOnConstructedSweeps`,
/// `testG7TwoWitnessAgreementOnTheProductionPath`, `testG7StalePoseSentinel`,
/// `testG8SecularDriftScreen`,
/// `testG9MirrorCheckIsSensitiveToAOneSidedError`.
final class MuscleLengthModeTests: XCTestCase {

    // MARK: - Registered populations

    /// The five single-DOF sweep families in the OpenSim fixture. Poses inside a
    /// family are consecutive, so adjacent pairs within a family are exactly the
    /// registered 111 (knee 28, hip 28, ankle 28, elbow 15, shoulder 12).
    static let sweepFamilies = ["knee_sweep_", "hip_sweep_", "ankle_sweep_",
                                "elbow_sweep_", "shoulder_sweep_"]

    static let scoredClips = ["video_012", "video_015"]

    // MARK: - The registered bars, frozen

    /// The bars exactly as registered on 2026-08-13. NOTHING here moved in the
    /// 2026-08-14 conversion: every converted gate still asserts its own bar
    /// value out of this enum before it records what the measurement did
    /// against it, so weakening a bar to make a conversion "agree" breaks the
    /// conversion instead of hiding inside it.
    enum RegisteredBar {
        /// G2(a): flicker rate `<= 1.0 %`.
        static let g2FlickerRate = 0.01
        /// G2(b): directional fraction `>= 40 %`.
        static let g2DirectionalFraction = 0.40
        /// G2(c): per-capsule directional fraction `>= 10 %`.
        static let g2CapsuleDirectionalFraction = 0.10
        /// G2(d): defined fraction `>= 90 %`.
        static let g2DefinedFraction = 0.90
        /// G2(e): grey-transition rate `<= 2.0 %`.
        static let g2GreyTransitionRate = 0.02
        /// G2(f): `>= 300` defined muscle-frames per clip.
        static let g2DefinedSamples = 300
        /// G3(v): identified iff `nullFraction <= 0.5`. Pinned against the
        /// shipped constant, which is `PostureFindings.depthSuppressionFraction`.
        static let g3IdentifiedNullFractionCeiling = 0.5
        /// G4(a): every registered direction matches on `>= 95 %` of the pairs
        /// clearing the deadband.
        static let g4DirectionMatch = 0.95
        /// G7(a): `>= 99.0 %` two-witness agreement …
        static let g7WitnessAgreement = 0.99
        /// … over `>= 500` jointly-clearing muscle-frames per clip.
        static let g7WitnessJointFrames = 500
        /// G7(b): an admitted muscle's `L_MT` range must exceed 1e-6 m.
        static let g7MinimumLengthRangeMetres = 1.0e-6
        /// G7(b): re-imposing a stored pose must reproduce `L_MT` to 1e-9 m.
        static let g7ReimposedPoseDeltaMetres = 1.0e-9
        /// G8(a) primary: trend excursion `<= 30 %` of range for every muscle.
        static let g8PrimaryExcursion = 0.30
        /// G8(a) secondary: `<= 20 %` for at least 90 % of them.
        static let g8SecondaryExcursion = 0.20
        /// G8(a) secondary: at most 10 % of the scored muscles may exceed it.
        static let g8SecondaryFraction = 0.10
        /// G9(b): the perturbed re-run must produce at least this many
        /// disagreements, i.e. it must FAIL clause (a).
        static let g9RequiredDisagreements = 1
    }

    /// What reopening this layer costs, stated once and printed by every
    /// converted gate so no single receipt can be read as a partial pass.
    static let reopeningRequirement =
        "production-grade 20-marker solved-pose clip fixture + fresh adjudication"

    /// The one place the conversion's authority is written down.
    static let conversionAuthority = "2026-08-14 owner-authorised, STATUS next-step 41 (a)"

    /// Records a converted gate's adjudicated outcome as a greppable line. The
    /// numbers themselves are asserted by the caller; this is the receipt that
    /// makes the verdict findable next to them.
    func recordFailedGate(_ gate: String, clip: String? = nil,
                          measured: String, bar: String, why: String) {
        print("MODE-VERDICT gate=\(gate)"
              + (clip.map { " clip=\($0)" } ?? "")
              + " outcome=FAILED_AGAINST_REGISTERED_BAR"
              + " measured=[\(measured)] registered_bar=[\(bar)] because=\(why)"
              + " layer=NOT_SHIPPED ui_wired=false"
              + " reopening_requires=[\(Self.reopeningRequirement)]"
              + " conversion=[\(Self.conversionAuthority)]")
    }

    // MARK: - Shared model context

    /// One loaded model + parsed muscle paths + the derived Rule-0 census,
    /// built once for the whole class. Building it twice would double the
    /// heaviest fixed cost in the file for no evidence.
    final class ModelContext {
        let bridge: NimbleBridge
        let computer: MomentArmComputer
        let dofNames: [String]
        let muscleNames: [String]
        let muscleIndexByName: [String: Int]
        let resolutions: [MuscleObservabilityMask.CapsuleResolution]
        /// The 32 displayed model muscles: 24 capsule-resolved + the family heads.
        let displayedMuscles: [String]
        let table: OpenSimReferenceFixture.Table
        let osimText: String

        init(bundle: Bundle) throws {
            let path = try XCTUnwrap(
                bundle.path(forResource: "FullBody", ofType: "osim")
                    ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
                "no FullBody.osim in the bundle")
            bridge = NimbleBridge()
            guard bridge.loadModel(fromPath: path) else {
                throw NSError(domain: "MuscleLengthModeTests", code: 1)
            }
            computer = MomentArmComputer()
            guard computer.parseMusclePaths(fromOsimPath: path, from: bridge) else {
                throw NSError(domain: "MuscleLengthModeTests", code: 2)
            }
            dofNames = bridge.dofNames
            muscleNames = computer.muscleNames
            muscleIndexByName = Dictionary(uniqueKeysWithValues:
                muscleNames.enumerated().map { ($0.element, $0.offset) })
            osimText = try String(contentsOfFile: path, encoding: .utf8)

            // Rule 0, derived from the artifact. The capsule names come from the
            // overlay's own anatomical list; nothing here names a muscle.
            let capsules = MuscleOverlay.muscleDefs.map(\.name)
            var seen = Set<String>()
            let orderedCapsules = capsules.filter { seen.insert($0).inserted }
            let pathPoints = MuscleObservabilityMask.pathPointNamesByMuscle(osimText: osimText)
            resolutions = MuscleObservabilityMask.resolve(capsules: orderedCapsules,
                                                          pathPointNames: pathPoints)
            displayedMuscles = resolutions.flatMap(\.modelMuscles)

            table = try OpenSimReferenceFixture.load(bundle: bundle)
        }

        func indices(_ names: [String]) -> [NSNumber] {
            names.compactMap { muscleIndexByName[$0].map(NSNumber.init(value:)) }
        }

        func dofIndices(_ names: [String]) -> [NSNumber] {
            names.compactMap { dofNames.firstIndex(of: $0).map(NSNumber.init(value:)) }
        }

        /// `R` for a named block, as `[muscle][coordinate]`, at the given pose.
        /// Also leaves the shared skeleton AT that pose, which is what lets the
        /// marker Jacobian be read at the same configuration.
        func momentArms(pose: [Double], muscles: [String], coordinates: [String]) -> [[Double]]? {
            let mIdx = indices(muscles)
            let cIdx = dofIndices(coordinates)
            guard mIdx.count == muscles.count, cIdx.count == coordinates.count else { return nil }
            guard let flat = computer.computeMomentArms(
                withJointAngles: pose.map(NSNumber.init(value:)),
                dofNames: dofNames,
                muscleIndices: mIdx,
                coordinateIndices: cIdx
            ) else { return nil }
            guard flat.count == muscles.count * coordinates.count else { return nil }
            return (0..<muscles.count).map { m in
                (0..<coordinates.count).map { c in flat[m * coordinates.count + c].doubleValue }
            }
        }

        /// Park the shared skeleton at `pose` without computing anything. The
        /// subset moment-arm call with empty subsets is exactly a pose setter.
        @discardableResult
        func setPose(_ pose: [Double]) -> Bool {
            computer.computeMomentArms(withJointAngles: pose.map(NSNumber.init(value:)),
                                       dofNames: dofNames,
                                       muscleIndices: [],
                                       coordinateIndices: []) != nil
        }
    }

    private static let sharedContext: Result<ModelContext, Error> = {
        Result { try ModelContext(bundle: Bundle(for: MuscleLengthModeTests.self)) }
    }()

    private func context() throws -> ModelContext { try Self.sharedContext.get() }

    // MARK: - Fixture-face helpers

    /// `σ̂` on the FIXTURE face: frozen at 1e-6 rad for every coordinate.
    private func fixtureNoise(_ n: Int) -> [Double] {
        [Double](repeating: MuscleLengthModeClassifier.fixtureFaceJointNoiseRadians, count: n)
    }

    private static let velocityGain =
        WindowedDerivativeFilter.velocityNoiseGain(taps: MuscleLengthModeClassifier.taps)

    /// The union of the fixture-declared spans of a muscle list.
    private func spanUnion(_ names: [String], table: OpenSimReferenceFixture.Table) -> [String] {
        var union: [String] = []
        var seen = Set<String>()
        for name in names {
            guard let index = table.muscleIndex(name) else { continue }
            for coordinate in table.muscles[index].coordinates where seen.insert(coordinate).inserted {
                union.append(coordinate)
            }
        }
        return union
    }

    /// Adjacent pose pairs inside the registered single-DOF sweep families.
    private func sweepPairs(_ table: OpenSimReferenceFixture.Table) -> [(a: Int, b: Int, coordinate: String)] {
        var out: [(Int, Int, String)] = []
        for i in 0..<(table.poses.count - 1) {
            let first = table.poses[i].id
            let second = table.poses[i + 1].id
            guard let family = Self.sweepFamilies.first(where: { first.hasPrefix($0) }),
                  second.hasPrefix(family) else { continue }
            let a = table.poses[i].values
            let b = table.poses[i + 1].values
            let moved = (0..<a.count).filter { abs(a[$0] - b[$0]) > 1e-9 }
            guard moved.count == 1 else { continue }
            out.append((i, i + 1, table.coordinateNames[moved[0]]))
        }
        return out
    }

    // MARK: - G1: sign correctness against the validated reference

    /// G1 HEADLINE. `>= 99.0 %` sign agreement over the 14 scoreable
    /// non-multi-wrap displayed muscles; ZERO disagreements at `>= 10x` the
    /// deadband over a population of `>= 250` cells AND `>= 25 %` of scored
    /// cells; coverage `>= 60 %` of spanning cells.
    func testG1SignAgreementAgainstTheOpenSimLengthOracle() throws {
        let ctx = try context()
        let outcome = try g1Outcome(ctx: ctx, multiWrap: false)
        printG1("headline", outcome)
        XCTAssertGreaterThanOrEqual(outcome.agreement, 0.99,
            "G1(a): sign agreement against lengthWrapOn")
        XCTAssertGreaterThanOrEqual(outcome.strongCells, 250,
            "G1(b): the >=10x population is under-powered")
        XCTAssertGreaterThanOrEqual(Double(outcome.strongCells), 0.25 * Double(outcome.scoredCells),
            "G1(b): the >=10x population is under 25 % of scored cells")
        XCTAssertEqual(outcome.strongDisagreements, 0,
            "G1(b): a sign disagreement an order of magnitude above the deadband")
        XCTAssertGreaterThanOrEqual(outcome.coverage, 0.60, "G1(c): coverage floor")
    }

    /// G1 MULTI-WRAP, scored against the ANALYTIC column rather than the
    /// reference's own length, because OpenSim's multi-wrap length is provably
    /// not a path length there while its analytic moment arm never touches that
    /// bookkeeping.
    func testG1MultiWrapSignAgreementAgainstTheAnalyticColumn() throws {
        let ctx = try context()
        let outcome = try g1Outcome(ctx: ctx, multiWrap: true)
        printG1("multiwrap", outcome)
        XCTAssertGreaterThanOrEqual(outcome.agreement, 0.99, "G1 multi-wrap: sign agreement")
        XCTAssertGreaterThanOrEqual(outcome.strongCells, 40, "G1 multi-wrap: >=10x under-powered")
        XCTAssertGreaterThanOrEqual(Double(outcome.strongCells), 0.25 * Double(outcome.scoredCells),
            "G1 multi-wrap: >=10x under 25 % of scored cells")
        XCTAssertEqual(outcome.strongDisagreements, 0, "G1 multi-wrap: strong-cell disagreement")
        XCTAssertGreaterThanOrEqual(outcome.coverage, 0.60, "G1 multi-wrap: coverage floor")
    }

    private struct G1Outcome {
        var muscles = 0
        var spanningCells = 0
        var scoredCells = 0
        var agreements = 0
        var strongCells = 0
        var strongDisagreements = 0
        var rightSideCells = 0
        var leftSideCells = 0
        /// Reported, not gated: which muscles carry the disagreements, and at
        /// which sweep midpoints. "The worst X is on muscle M" has already cost
        /// this repo a wrong investigation once.
        var disagreementsByMuscle: [String: Int] = [:]
        var strongDisagreementDetail: [String] = []
        var agreement: Double { scoredCells == 0 ? 0 : Double(agreements) / Double(scoredCells) }
        var coverage: Double { spanningCells == 0 ? 0 : Double(scoredCells) / Double(spanningCells) }
    }

    private func printG1(_ label: String, _ o: G1Outcome) {
        print("MODE-METRIC g1 face=\(label) muscles=\(o.muscles) spanning=\(o.spanningCells) "
              + "scored=\(o.scoredCells) agree=\(o.agreements) "
              + String(format: "agreement=%.6f", o.agreement)
              + " strong=\(o.strongCells) strong_disagree=\(o.strongDisagreements) "
              + String(format: "coverage=%.6f", o.coverage)
              + " right_cells=\(o.rightSideCells) left_cells=\(o.leftSideCells)"
              + " by_muscle=[" + o.disagreementsByMuscle.sorted { $0.value > $1.value }
                  .map { "\($0.key)=\($0.value)" }.joined(separator: ",") + "]"
              + " strong_detail=[" + o.strongDisagreementDetail.prefix(10).joined(separator: ",") + "]")
    }

    private func g1Outcome(ctx: ModelContext, multiWrap: Bool) throws -> G1Outcome {
        let table = ctx.table
        let pairs = sweepPairs(table)
        XCTAssertEqual(pairs.count, 111, "the registered single-DOF sweep population")

        // Scoreable = displayed AND present in the fixture AND on the requested
        // wrap-class side of the multi-wrap split.
        let scoreable = ctx.displayedMuscles.filter { name in
            guard let index = table.muscleIndex(name) else { return false }
            let isMulti = ctx.computer.pathWrapCount(forMuscleNamed: name) >= 2
            guard isMulti == multiWrap else { return false }
            // Only muscles that span at least one swept coordinate can score.
            return table.muscles[index].coordinates.contains { coordinate in
                pairs.contains { $0.coordinate == coordinate }
            }
        }
        var outcome = G1Outcome()
        outcome.muscles = scoreable.count
        guard !scoreable.isEmpty else { return outcome }

        let coordinates = spanUnion(scoreable, table: table)
        let noise = fixtureNoise(coordinates.count)

        for pair in pairs {
            // The fixture's coordinate order and the LIVE model's DOF order
            // differ in 160 of 169 positions, so the pose has to be re-indexed
            // by NAME before it is handed to the solver. Feeding the fixture's
            // own vector straight through scrambled every pose and produced a
            // moment arm that did not vary with the swept angle at all.
            let qa = table.poses[pair.a].values
            let qb = table.poses[pair.b].values
            let poseA = Self.orderedPose(ctx: ctx, table: table, poseIndex: pair.a)
            let poseB = Self.orderedPose(ctx: ctx, table: table, poseIndex: pair.b)
            let mid = zip(poseA, poseB).map { 0.5 * ($0 + $1) }
            guard let rows = ctx.momentArms(pose: mid, muscles: scoreable,
                                            coordinates: coordinates) else {
                XCTFail("moment arms failed at sweep pair \(pair.a)->\(pair.b)")
                continue
            }
            let dq = coordinates.map { name -> Double in
                guard let j = table.coordinateNames.firstIndex(of: name) else { return 0 }
                return qb[j] - qa[j]
            }

            for (m, name) in scoreable.enumerated() {
                guard let index = table.muscleIndex(name),
                      table.muscles[index].coordinates.contains(pair.coordinate) else { continue }
                outcome.spanningCells += 1
                if name.hasSuffix("_r") { outcome.rightSideCells += 1 } else { outcome.leftSideCells += 1 }

                let deadband = MuscleLengthModeClassifier.stepDeadbandMetres(
                    momentArmRow: rows[m], jointNoiseRadians: noise,
                    velocityNoiseGain: Self.velocityGain)

                let reference: Double
                if multiWrap {
                    // The analytic column at the two bracketing poses, averaged
                    // to the midpoint the port is evaluated at. Only one
                    // coordinate moves, so this is a scalar sign statement.
                    guard let rowA = table.row(pose: pair.a, muscle: index),
                          let rowB = table.row(pose: pair.b, muscle: index),
                          let k = table.muscles[index].coordinates.firstIndex(of: pair.coordinate),
                          k < rowA.momentArmsWrapOn.count, k < rowB.momentArmsWrapOn.count,
                          let j = table.coordinateNames.firstIndex(of: pair.coordinate)
                    else { continue }
                    let analytic = 0.5 * (rowA.momentArmsWrapOn[k] + rowB.momentArmsWrapOn[k])
                    reference = -analytic * (qb[j] - qa[j])
                } else {
                    guard let rowA = table.row(pose: pair.a, muscle: index),
                          let rowB = table.row(pose: pair.b, muscle: index) else { continue }
                    reference = rowB.lengthWrapOn - rowA.lengthWrapOn
                }

                guard abs(reference) > deadband else { continue }
                outcome.scoredCells += 1

                let ours = MuscleLengthModeClassifier.lengthRate(momentArmRow: rows[m],
                                                                jointVelocity: dq)
                let agrees = (ours > 0) == (reference > 0)
                if agrees { outcome.agreements += 1 } else {
                    outcome.disagreementsByMuscle[name, default: 0] += 1
                }
                if abs(reference) >= 10.0 * deadband {
                    outcome.strongCells += 1
                    if !agrees {
                        outcome.strongDisagreements += 1
                        if outcome.strongDisagreementDetail.count < 10,
                           let j = table.coordinateNames.firstIndex(of: pair.coordinate) {
                            outcome.strongDisagreementDetail.append(
                                String(format: "%@@%@=%.2fdeg(ours=%.3emm,ref=%.3emm)",
                                       name, pair.coordinate,
                                       0.5 * (qa[j] + qb[j]) * 180.0 / .pi,
                                       ours * 1000.0, reference * 1000.0))
                        }
                    }
                }
            }
        }
        return outcome
    }

    // MARK: - G3: the mask is drive-aware, derived, sound and non-empty

    /// G3(i). The mechanism is DERIVED, and the measured-decision receipt it
    /// cites survives. Comment-stripped source for the negative half, RAW source
    /// for the positive half, so satisfying the negative by deleting the receipt
    /// fails instead.
    func testG3MaskMechanismIsDerivedNotAHandPickedList() throws {
        let ctx = try context()
        let repoRoot = Self.repositoryRoot()
        let scanned = [
            "BioMotion/Muscle/MuscleLengthMode.swift",
            "BioMotion/Muscle/MuscleObservabilityMask.swift",
            "BioMotion/ARKit/MuscleOverlay.swift",
            "BioMotion/Nimble/NimbleEngine.swift",
        ]
        var strippedByPath: [String: String] = [:]
        var rawByPath: [String: String] = [:]
        for relative in scanned {
            let url = repoRoot.appendingPathComponent(relative)
            let raw = try String(contentsOf: url, encoding: .utf8)
            rawByPath[relative] = raw
            strippedByPath[relative] = Self.strippingComments(raw)
        }

        for (relative, stripped) in strippedByPath {
            XCTAssertFalse(stripped.contains("structurallyUnreachableCoordinates"),
                           "\(relative) reaches for the test-only unreachable-coordinate list")
            XCTAssertFalse(stripped.contains("FullBodyDOFFixture"),
                           "\(relative) reaches for the test-only DOF fixture")
        }

        let maskSource = try XCTUnwrap(strippedByPath["BioMotion/Muscle/MuscleObservabilityMask.swift"])
        var nameHits = 0
        for muscle in ctx.displayedMuscles where maskSource.contains(muscle) { nameHits += 1 }
        XCTAssertEqual(nameHits, 0, "the mask names muscles instead of deriving them")

        let engineRaw = try XCTUnwrap(rawByPath["BioMotion/Nimble/NimbleEngine.swift"])
        let receipts = engineRaw.components(
            separatedBy: "FullBodyDOFFixture.structurallyUnreachableCoordinates").count - 1
        XCTAssertGreaterThanOrEqual(receipts, 1,
            "the measured-decision receipt was deleted to satisfy the negative half")

        XCTAssertLessThan(MuscleObservabilityMask.visibleSingularValueMetresPerRadian,
                          MuscleObservabilityMask.mustNotMaskColumnNormMetresPerRadian,
                          "sigma_visible must sit below the measured must-not-mask column norm")
        print("MODE-METRIC g3i files=\(scanned.count) muscle_name_hits=\(nameHits) "
              + "receipts=\(receipts) "
              + String(format: "sigma_visible=%.6f", MuscleObservabilityMask.visibleSingularValueMetresPerRadian))
    }

    /// G3(ii). At the FULL 20-marker set at the fixture's neutral pose, every
    /// structurally unreachable coordinate is UNIDENTIFIED and the three named
    /// lower-limb coordinates are IDENTIFIED. A superset check: with 60 rows
    /// against 169 coordinates the numerical null space has dimension >= 109, so
    /// an equality requirement would be unsatisfiable for any rank criterion.
    func testG3CriterionConsistencyAtTheFullMarkerSet() throws {
        let ctx = try context()
        let neutral = try XCTUnwrap(ctx.table.poseIndex("neutral"))
        let pose = Self.orderedPose(ctx: ctx, table: ctx.table, poseIndex: neutral)
        XCTAssertTrue(ctx.setPose(pose))

        let markers = JointMapping.primary.map(\.opensimName)
        let fractions = try XCTUnwrap(Self.nullFractions(ctx: ctx, markers: markers),
                                      "the 20-marker Jacobian did not resolve")

        var misclassified: [String] = []
        for name in FullBodyDOFFixture.structurallyUnreachableCoordinates {
            guard let j = ctx.dofNames.firstIndex(of: name) else { continue }
            if MuscleObservabilityMask.isIdentified(nullFraction: fractions[j]) {
                misclassified.append(name)
            }
        }
        XCTAssertEqual(misclassified, [], "structurally unreachable coordinates read as identified")

        for name in ["hip_flexion_r", "knee_angle_r", "ankle_angle_r"] {
            let j = try XCTUnwrap(ctx.dofNames.firstIndex(of: name))
            XCTAssertTrue(MuscleObservabilityMask.isIdentified(nullFraction: fractions[j]),
                          "\(name) must be identified at the full marker set")
            print(String(format: "MODE-METRIC g3ii coordinate=%@ null_fraction=%.6f", name, fractions[j]))
        }
        if let j = ctx.dofNames.firstIndex(of: "shoulder_rot_r") {
            print(String(format: "MODE-METRIC g3ii coordinate=shoulder_rot_r null_fraction=%.6f "
                         + "(reported, not gated)", fractions[j]))
        }
        print("MODE-METRIC g3ii unreachable=\(FullBodyDOFFixture.structurallyUnreachableCoordinates.count) "
              + "misclassified=\(misclassified.count)")
    }

    /// G3(iii). The runtime span predicate reproduces the fixture's own declared
    /// spans EXACTLY for all 32 displayed muscles.
    func testG3RuntimeSpansReproduceTheFixtureDeclaredSpans() throws {
        let ctx = try context()
        let table = ctx.table
        let neutral = try XCTUnwrap(table.poseIndex("neutral"))
        let pose = Self.orderedPose(ctx: ctx, table: table, poseIndex: neutral)
        let displayed = ctx.displayedMuscles
        let rows = try XCTUnwrap(ctx.momentArms(pose: pose, muscles: displayed,
                                                coordinates: ctx.dofNames))
        var mismatches: [String] = []
        for (m, name) in displayed.enumerated() {
            guard let index = table.muscleIndex(name) else {
                mismatches.append("\(name):absent-from-fixture")
                continue
            }
            let runtime = Set(MuscleObservabilityMask.spannedCoordinates(momentArmRow: rows[m])
                .map { ctx.dofNames[$0] })
            let declared = Set(table.muscles[index].coordinates)
            if runtime != declared {
                mismatches.append("\(name):+\(runtime.subtracting(declared).sorted())"
                                  + "-\(declared.subtracting(runtime).sorted())")
            }
        }
        print("MODE-METRIC g3iii displayed=\(displayed.count) mismatches=\(mismatches.count) "
              + "detail=\(mismatches.prefix(6).joined(separator: ","))")
        XCTAssertEqual(mismatches, [], "runtime spans differ from the fixture's declared spans")
    }

    /// G3(iv), ACCEPTANCE INVERTED. On both pinned clips the unidentified
    /// lower-limb set must be NON-EMPTY at every warmed frame, and all 12
    /// hip-spanning capsules must be suppressed by the fail-closed clip verdict.
    /// "Fires on 0" is DISPROOF.
    func testG3TheDriveAwareMaskIsNonEmptyOnThePinnedClips() throws {
        for clip in Self.scoredClips {
            let traversal = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g3iv clip=\(clip) warmed=\(traversal.warmedCount) "
                  + "empty_frames=\(traversal.framesWithEmptyUnidentifiedLowerLimb) "
                  + "unidentified_lower_limb=\(traversal.clipUnidentifiedLowerLimb.sorted()) "
                  + "hip_capsules=\(traversal.hipCapsules.count) "
                  + "hip_suppressed=\(traversal.suppressedHipCapsules.count) "
                  + "admitted=\(traversal.admittedCapsules.sorted())")
            XCTAssertEqual(traversal.framesWithEmptyUnidentifiedLowerLimb, 0,
                "G3(iv-a): the drive-aware mask fired on nothing at some warmed frame of \(clip)")
            XCTAssertEqual(traversal.suppressedHipCapsules.count, traversal.hipCapsules.count,
                "G3(iv-b): a hip-spanning capsule survived the fail-closed clip verdict on \(clip)")
        }
    }

    /// G3(v). The two-sided guard that stops "suppress everything" passing,
    /// deliberately anchored at DETERMINISTIC fixture poses rather than on clip
    /// frames, because the fail-closed clip verdict is legitimately vulnerable to
    /// incidental passage through the near-extension singularity.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// The bar is UNCHANGED and still asserted: identified iff
    /// `nullFraction <= 0.5`, read from the SHIPPED constant, so widening
    /// `sigma_visible`'s companion crossover to make the knee "identified"
    /// breaks this test rather than passing it. What changed is that the method
    /// now RECORDS the measured null fractions instead of demanding they clear
    /// the crossover. Pins from `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log`
    /// lines 8176-8181 (`grep 'MODE-METRIC g3v'`). Each pin is bidirectional —
    /// a change to the mask, the fixtures, the classifier or the model moves
    /// these six numbers and turns this test RED in EITHER direction, which
    /// forces a fresh adjudication instead of a silent re-baseline.
    /// VERDICT: G3(v) FAILED against its registered bar (`neutral` reads
    /// 0.832149 > 0.5 on both knees). The layer is NOT SHIPPED. Reopening needs
    /// richer fixtures — a production-grade 20-marker solved-pose clip — plus a
    /// fresh adjudication.
    func testG3KneeIsIdentifiedUnderTheFiveMarkerDriveAtFixturePoses() throws {
        let ctx = try context()
        let markers = try Self.clipMarkerNames(clip: Self.scoredClips[0], context: ctx)
        var failures: [String] = []
        var measured: [String: Double] = [:]
        for poseId in ["neutral", "run_1_midstance", "run_4_mid_swing"] {
            let poseIndex = try XCTUnwrap(ctx.table.poseIndex(poseId))
            let pose = Self.orderedPose(ctx: ctx, table: ctx.table, poseIndex: poseIndex)
            XCTAssertTrue(ctx.setPose(pose))
            let fractions = try XCTUnwrap(Self.nullFractions(ctx: ctx, markers: markers))
            for name in ["knee_angle_r", "knee_angle_l"] {
                let j = try XCTUnwrap(ctx.dofNames.firstIndex(of: name))
                print(String(format: "MODE-METRIC g3v pose=%@ coordinate=%@ null_fraction=%.6f",
                             poseId, name, fractions[j]))
                measured["\(poseId)/\(name)"] = fractions[j]
                if !MuscleObservabilityMask.isIdentified(nullFraction: fractions[j]) {
                    failures.append("\(poseId)/\(name)")
                }
            }
        }

        // The registered bar, asserted where the gate reads it.
        XCTAssertEqual(MuscleObservabilityMask.identifiedNullFractionCeiling,
                       RegisteredBar.g3IdentifiedNullFractionCeiling,
                       "G3(v)'s crossover moved: the registered decision boundary is 0.5")

        // The MEASURED outcome, pinned to the receipt.
        let pinned: [String: Double] = [
            "neutral/knee_angle_r": 0.832149,
            "neutral/knee_angle_l": 0.832149,
            "run_1_midstance/knee_angle_r": 0.297860,
            "run_1_midstance/knee_angle_l": 0.113131,
            "run_4_mid_swing/knee_angle_r": 0.113131,
            "run_4_mid_swing/knee_angle_l": 0.297860,
        ]
        XCTAssertEqual(Set(measured.keys), Set(pinned.keys),
                       "the G3(v) population itself moved")
        for (key, expected) in pinned {
            let value = try XCTUnwrap(measured[key])
            XCTAssertEqual(value, expected, accuracy: 1.0e-6,
                           "G3(v) pin \(key): the measured null fraction moved off its receipt")
        }

        // The VERDICT: the gate failed, and it failed at exactly one pose.
        XCTAssertEqual(failures, ["neutral/knee_angle_r", "neutral/knee_angle_l"],
            "G3(v) FAILED at `neutral` and only at `neutral`; both running poses are identified")
        for running in ["run_1_midstance", "run_4_mid_swing"] {
            for name in ["knee_angle_r", "knee_angle_l"] {
                let value = try XCTUnwrap(measured["\(running)/\(name)"])
                XCTAssertTrue(MuscleObservabilityMask.isIdentified(nullFraction: value),
                              "\(running)/\(name) must stay identified")
            }
        }
        let neutral = try XCTUnwrap(measured["neutral/knee_angle_r"])
        XCTAssertGreaterThan(neutral, RegisteredBar.g3IdentifiedNullFractionCeiling,
            "G3(v) is recorded as FAILED: the straight-leg pose sits ABOVE the crossover")
        recordFailedGate("G3(v)",
                         measured: String(format: "neutral=%.6f both knees; "
                                          + "run_1_midstance=0.297860/0.113131; "
                                          + "run_4_mid_swing=0.113131/0.297860", neutral),
                         bar: "identified iff null_fraction <= "
                              + "\(RegisteredBar.g3IdentifiedNullFractionCeiling)",
                         why: "a straight leg with no knee marker in the 5-marker drive")
    }

    /// G3(vi). The Rule-0 census, gated because the alias bug degrades SILENTLY:
    /// a bare exact-name lookup makes six capsules unresolved, hence not
    /// admitted, and every population in the battery shrinks WITH the bug rather
    /// than failing ON it.
    func testG3TheCapsuleCensusIsExactlyTheRegisteredDerivation() throws {
        let ctx = try context()
        let byKind = Dictionary(grouping: ctx.resolutions, by: \.kind).mapValues(\.count)
        let resolved = ctx.resolutions.filter(\.isResolved)
        print("MODE-METRIC g3vi capsules=\(ctx.resolutions.count) "
              + "exact=\(byKind[.exact] ?? 0) alias=\(byKind[.alias] ?? 0) "
              + "zero=\(byKind[.zero] ?? 0) multi=\(byKind[.multi] ?? 0) "
              + "resolved=\(resolved.count) displayed=\(ctx.displayedMuscles.count) "
              + "unresolved=\(ctx.resolutions.filter { !$0.isResolved }.map(\.capsule).sorted())")
        XCTAssertEqual(ctx.resolutions.count, 26)
        XCTAssertEqual(byKind[.exact] ?? 0, 18, "EXACT census")
        XCTAssertEqual(byKind[.alias] ?? 0, 6, "ALIAS census")
        XCTAssertEqual(byKind[.zero] ?? 0, 2, "ZERO census")
        XCTAssertEqual(byKind[.multi] ?? 0, 0, "MULTI census")
        XCTAssertEqual(resolved.count, 24, "capsule-resolved count")
        XCTAssertEqual(ctx.displayedMuscles.count, 32, "displayed model muscle count")
        XCTAssertEqual(Set(ctx.displayedMuscles).count, 32, "displayed set must be distinct")
    }

    // MARK: - G4: physiology smoke direction check

    private struct SweepSpec {
        let name: String
        let coordinate: String
        let fromDegrees: Double
        let toDegrees: Double
        /// Muscles that must LENGTHEN, then muscles that must SHORTEN.
        let lengthening: [String]
        let shortening: [String]
    }

    /// FROZEN before any result. The exclusions are frozen too, because post-hoc
    /// exclusion of a failed anchor is the classic laundering route: tfl (IT-band
    /// mediated), the adductor group, glmed*/glmin*, piri, sart/grac on the hip
    /// sweep, and the ENTIRE shoulder sweep.
    private static let g4Sweeps: [SweepSpec] = [
        SweepSpec(name: "knee_flexion", coordinate: "knee_angle_r",
                  fromDegrees: 0, toDegrees: 140,
                  lengthening: ["recfem_r", "vasint_r", "vaslat140_r", "vasmed_r"],
                  shortening: ["semimem_r", "semiten_r", "bflh140_r", "bfsh140_r",
                               "grac_r", "sart_r"]),
        SweepSpec(name: "ankle_dorsiflexion", coordinate: "ankle_angle_r",
                  fromDegrees: -30, toDegrees: 20,
                  lengthening: ["soleus_r", "gasmed_r", "gaslat140_r"],
                  shortening: ["tibant_r"]),
        SweepSpec(name: "hip_flexion", coordinate: "hip_flexion_r",
                  fromDegrees: -20, toDegrees: 120,
                  lengthening: ["glmax1_r", "glmax2_r", "glmax3_r"],
                  shortening: ["iliacus_r", "psoas_r"]),
        SweepSpec(name: "elbow_flexion", coordinate: "elbow_flex_r",
                  fromDegrees: 0, toDegrees: 150,
                  lengthening: ["TRIlong_r", "TRImed_r", "TRIlat_r", "ANC_r"],
                  shortening: ["BIClong_r", "BICshort_r", "BRD_r"]),
    ]

    /// G4(a)+(b). Every registered direction matches on `>= 95 %` of that
    /// sweep's adjacent pose pairs clearing the deadband, and ZERO registered
    /// muscles are majority-wrong.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// The 95 % bar is UNCHANGED and still asserted through `RegisteredBar`, so
    /// lowering it to 0.866667 breaks this test. The registered sweep endpoints,
    /// the anchor set and the frozen exclusion list are untouched — shortening
    /// the elbow sweep to recover the triceps is exactly the laundering move the
    /// registration froze out, and it was not made. What changed is that the
    /// method now RECORDS the per-anchor match rates against that bar. Pins from
    /// `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log` lines 8205-8231
    /// (`grep 'MODE-METRIC g4'`): 22 anchors at 1.000000, three triceps at
    /// 0.866667 disagreeing at 132.5/137.5/142.5/147.5 deg, `bfsh140_r` at
    /// 0.964286 disagreeing at 137.5 deg — 22 + 3 + 1 = 26. Bidirectional: any
    /// change to the model, the classifier or the deadband moves these rates and
    /// turns the test RED in EITHER direction.
    /// VERDICT: G4(a) FAILED against its registered bar. G4(b) still PASSES and
    /// is still asserted as a bar. The layer is NOT SHIPPED. Reopening needs
    /// richer fixtures — a production-grade 20-marker solved-pose clip — plus a
    /// fresh adjudication (and, for this gate specifically, the owner-level
    /// registration decision in STATUS next-step 42).
    func testG4PhysiologyDirectionsOnConstructedSweeps() throws {
        let ctx = try context()
        let results = try g4Run(ctx: ctx, invertSign: false, rotateNames: false)
        var worst = 1.0
        var majorityWrong: [String] = []
        for r in results {
            print(String(format: "MODE-METRIC g4 sweep=%@ muscle=%@ pairs=%d cleared=%d match=%.6f",
                         r.sweep, r.muscle, r.pairs, r.cleared, r.rate)
                  + " disagree_at_deg=[" + r.disagreeingMidpointsDegrees
                      .map { String(format: "%.1f", $0) }.joined(separator: ",") + "]")
            if r.cleared > 0 { worst = min(worst, r.rate) }
            if r.cleared > 0 && r.rate < 0.5 { majorityWrong.append("\(r.sweep)/\(r.muscle)") }
        }
        let anchors = results.count
        let scored = results.filter { $0.cleared > 0 }.count
        print("MODE-METRIC g4a anchors=\(anchors) scored=\(scored) "
              + String(format: "worst=%.6f", worst))

        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g4DirectionMatch, 0.95,
                       "G4(a)'s registered bar is 95 % and did not move")

        // Clause (a)'s power precondition and clause (b) both still HOLD, so
        // they stay ordinary assertions rather than pins.
        XCTAssertEqual(scored, anchors, "an anchor cleared the deadband on no pair at all")
        XCTAssertEqual(majorityWrong, [], "G4(b): a registered muscle is majority-wrong")
        XCTAssertTrue(results.allSatisfy { $0.cleared == $0.pairs },
                      "all 26 anchors cleared the deadband on every pair")

        // The MEASURED outcome, pinned to the receipt.
        XCTAssertEqual(anchors, 26, "the G4 anchor population itself moved")
        var byKey: [String: G4Result] = [:]
        for r in results { byKey["\(r.sweep)/\(r.muscle)"] = r }
        XCTAssertEqual(byKey.count, 26, "two G4 anchors collided on one key")
        let pinnedBelowOne: [String: (rate: Double, disagreeAtDegrees: [Double])] = [
            "elbow_flexion/TRIlong_r": (0.866667, [132.5, 137.5, 142.5, 147.5]),
            "elbow_flexion/TRImed_r": (0.866667, [132.5, 137.5, 142.5, 147.5]),
            "elbow_flexion/TRIlat_r": (0.866667, [132.5, 137.5, 142.5, 147.5]),
            "knee_flexion/bfsh140_r": (0.964286, [137.5]),
        ]
        for (key, pin) in pinnedBelowOne {
            let r = try XCTUnwrap(byKey[key], "G4 pin \(key) is no longer a registered anchor")
            XCTAssertEqual(r.rate, pin.rate, accuracy: 1.0e-6,
                           "G4 pin \(key): the measured match rate moved off its receipt")
            XCTAssertEqual(r.disagreeingMidpointsDegrees.sorted().count,
                           pin.disagreeAtDegrees.count,
                           "G4 pin \(key): the disagreement COUNT moved")
            for (measuredDegrees, pinnedDegrees) in zip(r.disagreeingMidpointsDegrees.sorted(),
                                                        pin.disagreeAtDegrees) {
                XCTAssertEqual(measuredDegrees, pinnedDegrees, accuracy: 1.0e-9,
                               "G4 pin \(key): a disagreement moved off \(pinnedDegrees) deg")
            }
        }
        let perfect = results.filter { $0.rate == 1.0 }
        XCTAssertEqual(perfect.count, 22,
                       "22 anchors read 1.000000; 22 + 3 triceps + bfsh140_r = 26")
        XCTAssertEqual(perfect.count + pinnedBelowOne.count, anchors,
                       "the pinned split does not account for every anchor")

        // The VERDICT.
        XCTAssertEqual(worst, 0.866667, accuracy: 1.0e-6,
                       "G4(a)'s worst anchor moved off its receipt")
        XCTAssertLessThan(worst, RegisteredBar.g4DirectionMatch,
            "G4(a) is recorded as FAILED: the worst registered direction is below the 95 % bar")
        recordFailedGate("G4(a)",
                         measured: String(format: "worst=%.6f on TRIlong_r/TRImed_r/TRIlat_r "
                                          + "(4 of 30 elbow steps: 132.5/137.5/142.5/147.5 deg); "
                                          + "bfsh140_r=0.964286; 22 anchors at 1.000000", worst),
                         bar: ">= \(RegisteredBar.g4DirectionMatch) per anchor",
                         why: "FullBody.osim reverses the triceps moment arm at ~125-130 deg and "
                              + "the port reproduces the model — the frozen anchor disagrees with "
                              + "the shipped model, which is a registration question (next-step 42)")
    }

    /// G4(c) MAPPING CHECK. Muscles the fixture declares do NOT span the swept
    /// coordinate read the third state. Labelled a mapping check, not a
    /// discriminative control: for a non-spanning muscle the finite difference of
    /// an unchanging length returns exactly 0, so it reads the third state
    /// whether or not the layer is correct.
    func testG4NonSpanningMusclesReadTheThirdState() throws {
        let ctx = try context()
        let table = ctx.table
        var total = 0
        var third = 0
        for sweep in Self.g4Sweeps {
            let anchors = sweep.lengthening + sweep.shortening
            let nonSpanning = Set(Self.g4Sweeps.flatMap { $0.lengthening + $0.shortening })
                .subtracting(anchors)
                .filter { name in
                    guard let index = table.muscleIndex(name) else { return false }
                    return !table.muscles[index].coordinates.contains(sweep.coordinate)
                }
                .sorted()
            guard !nonSpanning.isEmpty else { continue }
            let outcome = try g4Sweep(ctx: ctx, sweep: sweep, muscles: nonSpanning,
                                      invertSign: false, rotateNames: false)
            for r in outcome {
                total += r.pairs
                third += r.pairs - r.cleared
            }
        }
        let rate = total == 0 ? 0 : Double(third) / Double(total)
        print("MODE-METRIC g4c pairs=\(total) third_state=\(third) "
              + String(format: "rate=%.6f", rate))
        XCTAssertGreaterThan(total, 0, "G4(c) scored no pairs")
        XCTAssertGreaterThanOrEqual(rate, 0.99, "G4(c): non-spanning muscles left the third state")
    }

    /// G4(d) DISCRIMINATION. Sign inversion and name rotation both leave every
    /// stability statistic untouched, so a gate blind to them is not evidence of
    /// correctness. BOTH re-runs must FAIL clause (a).
    func testG4DiscriminationControlsMustFail() throws {
        let ctx = try context()
        for (label, inverted, rotated) in [("sign_inverted", true, false),
                                           ("name_rotated", false, true)] {
            let results = try g4Run(ctx: ctx, invertSign: inverted, rotateNames: rotated)
            let scored = results.filter { $0.cleared > 0 }
            let worst = scored.map(\.rate).min() ?? 1.0
            print("MODE-METRIC g4d control=\(label) scored=\(scored.count) "
                  + String(format: "worst=%.6f", worst))
            XCTAssertLessThan(worst, 0.95,
                "G4(d): the \(label) control still passed clause (a) — the gate is not measuring what it claims")
        }
    }

    private struct G4Result {
        let sweep: String
        let muscle: String
        let pairs: Int
        let cleared: Int
        let matched: Int
        /// Reported, not gated: WHICH steps disagreed, in degrees of the swept
        /// coordinate at the pair midpoint. A disagreement clustered at one end
        /// of a sweep is a boundary-resolution reading; one spread through the
        /// middle is the signal being wrong.
        let disagreeingMidpointsDegrees: [Double]
        var rate: Double { cleared == 0 ? 0 : Double(matched) / Double(cleared) }
    }

    private func g4Run(ctx: ModelContext, invertSign: Bool, rotateNames: Bool) throws -> [G4Result] {
        var out: [G4Result] = []
        for sweep in Self.g4Sweeps {
            let muscles = sweep.lengthening + sweep.shortening
            out.append(contentsOf: try g4Sweep(ctx: ctx, sweep: sweep, muscles: muscles,
                                               invertSign: invertSign, rotateNames: rotateNames))
        }
        return out
    }

    private func g4Sweep(ctx: ModelContext, sweep: SweepSpec, muscles: [String],
                         invertSign: Bool, rotateNames: Bool) throws -> [G4Result] {
        let table = ctx.table
        let present = muscles.filter { ctx.muscleIndexByName[$0] != nil }
        XCTAssertEqual(present.count, muscles.count,
                       "a G4 anchor is not in the parsed model: \(Set(muscles).subtracting(present))")
        guard !present.isEmpty else { return [] }

        let coordinates = spanUnion(present, table: table)
        let noise = fixtureNoise(coordinates.count)
        let neutralIndex = try XCTUnwrap(table.poseIndex("neutral"))
        let base = Self.orderedPose(ctx: ctx, table: table, poseIndex: neutralIndex)
        let sweptIndex = try XCTUnwrap(ctx.dofNames.firstIndex(of: sweep.coordinate))
        let step = 5.0 * .pi / 180.0
        let from = sweep.fromDegrees * .pi / 180.0
        let to = sweep.toDegrees * .pi / 180.0
        let count = max(2, Int(((to - from) / step).rounded()) + 1)

        // Expected direction per muscle, BEFORE any control is applied.
        var expectation: [String: MuscleLengthMode] = [:]
        for name in sweep.lengthening { expectation[name] = .lengthening }
        for name in sweep.shortening { expectation[name] = .shortening }

        var pairs = [Int](repeating: 0, count: present.count)
        var cleared = [Int](repeating: 0, count: present.count)
        var matched = [Int](repeating: 0, count: present.count)
        var disagreements = [[Double]](repeating: [], count: present.count)

        for i in 0..<(count - 1) {
            let qa = from + Double(i) * step
            let qb = from + Double(i + 1) * step
            var poseA = base, poseB = base
            poseA[sweptIndex] = qa
            poseB[sweptIndex] = qb
            let mid = zip(poseA, poseB).map { 0.5 * ($0 + $1) }
            guard let rows = ctx.momentArms(pose: mid, muscles: present,
                                            coordinates: coordinates) else { continue }
            let dq = coordinates.map { $0 == sweep.coordinate ? (qb - qa) : 0.0 }

            for (m, _) in present.enumerated() {
                let deadband = MuscleLengthModeClassifier.stepDeadbandMetres(
                    momentArmRow: rows[m], jointNoiseRadians: noise,
                    velocityNoiseGain: Self.velocityGain)
                var value = MuscleLengthModeClassifier.lengthRate(momentArmRow: rows[m],
                                                                  jointVelocity: dq)
                if invertSign { value = -value }
                let mode = MuscleLengthModeClassifier.classify(value: value, deadband: deadband)
                pairs[m] += 1
                guard mode.isDirectional else { continue }
                cleared[m] += 1
                // Name rotation reassigns each measurement to its neighbour's
                // expectation, which is what a muscle-index misalignment does.
                let assigned = rotateNames ? present[(m + 1) % present.count] : present[m]
                if let expected = expectation[assigned], expected == mode {
                    matched[m] += 1
                } else {
                    disagreements[m].append(0.5 * (qa + qb) * 180.0 / .pi)
                }
            }
        }
        return present.enumerated().map { m, name in
            G4Result(sweep: sweep.name, muscle: name,
                     pairs: pairs[m], cleared: cleared[m], matched: matched[m],
                     disagreeingMidpointsDegrees: disagreements[m])
        }
    }

    // MARK: - G9: bilateral mirror coherence, narrowed

    /// G9(a). 100 % mode agreement across all 16 mirrored displayed-muscle pairs.
    ///
    /// SCOPE, BINDING: this detects LEFT/RIGHT INDEX MISALIGNMENT and ONE-SIDED
    /// PATH OR GEOMETRY ASYMMETRY at ONE SHARED POSE. It does NOT bound "the same
    /// wrap error evaluated at two different poses" — a mirrored pose puts both
    /// legs at the same configuration by construction — and that second sub-class
    /// IS the antiphase running regime. Registered stated limitation.
    func testG9BilateralMirrorCoherence() throws {
        let ctx = try context()
        let outcome = try g9Outcome(ctx: ctx, perturbationRelative: 0)
        print("MODE-METRIC g9a pairs=\(outcome.pairs) scored=\(outcome.scored) "
              + "disagree=\(outcome.disagreements) excluded=\(outcome.excluded) "
              + "detail=\(outcome.detail.prefix(6).joined(separator: ","))")
        XCTAssertGreaterThan(outcome.scored, 0, "G9(a) scored no steps")
        XCTAssertEqual(outcome.disagreements, 0, "G9(a): a mirrored pair disagreed in mode")
    }

    /// G9(b) DISCRIMINATION. The one-sided perturbation at the measured p99
    /// relative residual must FAIL clause (a), or the mirror check is not
    /// sensitive to the error class it exists to catch.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// The registered control is UNCHANGED — still the multiplicative
    /// `1 + 1.114 %` on one leg's row, still requiring at least one
    /// disagreement — and the requirement is still asserted through
    /// `RegisteredBar`. What changed is that the method now RECORDS that the
    /// control produced zero, together with the arithmetic proving it CANNOT
    /// produce anything else: a positive scalar multiple scales `-Rᵀdq` and
    /// `s_m` by the same factor, so both the sign and `|v| > D` are exactly
    /// invariant against a sign-only classifier. Pins from
    /// `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log` lines 8272 (g9a),
    /// 8275 (g9b) and 8276 (g9diag). Bidirectional: if the control ever DOES
    /// disagree, the pin goes red and the registration defect has to be
    /// re-adjudicated rather than quietly declared repaired.
    /// VERDICT: G9(b) FAILED against its registered bar, on a control that
    /// cannot discriminate. The layer is NOT SHIPPED. Reopening needs richer
    /// fixtures — a production-grade 20-marker solved-pose clip — plus a fresh
    /// adjudication (and, for this gate, the registration repair in STATUS
    /// next-step 43).
    func testG9MirrorCheckIsSensitiveToAOneSidedError() throws {
        let ctx = try context()
        let outcome = try g9Outcome(ctx: ctx, perturbationRelative: 0.01114)
        print("MODE-METRIC g9b form=multiplicative pairs=\(outcome.pairs) "
              + "scored=\(outcome.scored) disagree=\(outcome.disagreements)")
        // REPORTED, not gated, and printed BEFORE the registered assertion so the
        // receipt exists whatever that assertion does. The additive form is the
        // one that can change an answer at all; the multiplicative form the
        // registration named cannot — see the note on `g9Outcome`.
        let additive = try g9Outcome(ctx: ctx, perturbationRelative: 0, additiveRelative: 0.01114)
        print("MODE-METRIC g9diag form=additive_relative_row_norm rel=0.01114 "
              + "scored=\(additive.scored) disagree=\(additive.disagreements) "
              + "disagree_on_g9a_scored_steps=\(additive.disagreementsOnUnperturbedScoredSteps) "
              + "disagree_on_g9a_excluded_steps=\(additive.disagreementsOnUnperturbedExcludedSteps)")

        // The span union the perturbation is measured against, recomputed here
        // so the size arithmetic is checkable rather than asserted in prose.
        let rightMuscles = ctx.displayedMuscles.filter { $0.hasSuffix("_r") }.sorted()
        let rightCoords = spanUnion(rightMuscles, table: ctx.table)
        XCTAssertEqual(rightCoords.count, 6,
                       "the right-side span union moved; the companion's size factor is derived "
                       + "from it")
        let additiveSizeFactor = Double(rightCoords.count).squareRoot()
        print(String(format: "MODE-METRIC g9diag span_union=%d per_entry_rel=0.01114 "
                     + "row_norm_rel=%.6f factor_vs_registered=%.3f",
                     rightCoords.count, additiveSizeFactor * 0.01114, additiveSizeFactor))

        // The registered bar, asserted where the gate reads it.
        XCTAssertEqual(RegisteredBar.g9RequiredDisagreements, 1,
                       "G9(b) still requires the perturbed re-run to disagree at least once")

        // The MEASURED outcome, pinned to the receipt.
        XCTAssertEqual(outcome.pairs, 16, "the 16 displayed bilateral pairs moved")
        XCTAssertEqual(outcome.scored, 258, "G9(b) pin: the scored population moved")
        XCTAssertEqual(outcome.excluded, 270, "G9(b) pin: the excluded population moved")
        XCTAssertEqual(outcome.disagreements, 0,
                       "G9(b) pin: the multiplicative control is inert and must read 0")
        XCTAssertEqual(additive.scored, 528, "g9diag pin: the additive companion scores every step")
        XCTAssertEqual(additive.disagreements, 271, "g9diag pin: the companion's disagreements moved")
        XCTAssertEqual(additive.scored, outcome.scored + outcome.excluded,
                       "the companion's population is G9(a)'s scored PLUS G9(a)'s excluded, which "
                       + "is why its count is not comparable with G9(a)'s")
        XCTAssertEqual(additive.disagreementsOnUnperturbedScoredSteps
                       + additive.disagreementsOnUnperturbedExcludedSteps,
                       additive.disagreements, "the companion's split does not sum")
        // MEASURED, not inferred from the two totals: 270 of the companion's
        // 271 disagreements land on steps G9(a) never scores, so its count is
        // not comparable with G9(a)'s 258/0 and must never be quoted as if it
        // were the same population.
        XCTAssertEqual(additive.disagreementsOnUnperturbedExcludedSteps, 270,
                       "g9diag pin: disagreements on steps G9(a) EXCLUDES")
        XCTAssertEqual(additive.disagreementsOnUnperturbedScoredSteps, 1,
                       "g9diag pin: disagreements inside G9(a)'s own population")

        // The VERDICT.
        XCTAssertLessThan(outcome.disagreements, RegisteredBar.g9RequiredDisagreements,
            "G9(b) is recorded as FAILED: the registered multiplicative control produced no "
            + "disagreement, and it provably cannot — a positive scalar multiple is inert "
            + "against a sign-only classifier")
        recordFailedGate("G9(b)",
                         measured: "multiplicative scored=\(outcome.scored) "
                                   + "disagree=\(outcome.disagreements); ungated additive "
                                   + "companion scored=\(additive.scored) "
                                   + "disagree=\(additive.disagreements) of which "
                                   + "\(additive.disagreementsOnUnperturbedExcludedSteps) land on "
                                   + "steps G9(a) excludes",
                         bar: ">= \(RegisteredBar.g9RequiredDisagreements) disagreement",
                         why: "the registered control is inert by construction — a registration "
                              + "defect, not a layer defect (next-step 43); the companion is "
                              + String(format: "%.3f", additiveSizeFactor)
                              + "x the registered size and scores a different population")
    }

    private struct G9Outcome {
        var pairs = 0
        var scored = 0
        var disagreements = 0
        var excluded = 0
        var detail: [String] = []
        /// Of `disagreements`, how many land on steps the UNPERTURBED run
        /// SCORES, and how many on steps it EXCLUDES. Zero-cost on the
        /// unperturbed run (the two rows are the same array), and it is what
        /// makes "the companion diagnostic is not comparable with G9(a)" a
        /// MEASUREMENT rather than an inference from two totals.
        var disagreementsOnUnperturbedScoredSteps = 0
        var disagreementsOnUnperturbedExcludedSteps = 0
    }

    /// REPORTED, not gated. The registered discrimination control multiplies one
    /// leg's moment-arm row by `1 + 1.114 %`, and a positive scalar multiple is
    /// PROVABLY INERT against a sign-only classifier: it scales `-Rᵀdq` and
    /// `s_m` by the same factor, so the comparison `|v| > D` and the sign of `v`
    /// are both exactly invariant. That is a defect in the control's
    /// construction, not evidence about the layer, and it is why G9(b) reads
    /// zero. This companion applies a one-sided error of the SAME registered
    /// relative size ADDITIVELY — `±1.114 %` of the row's own L2 norm, signed
    /// against each entry — which is the form that can actually change an
    /// answer. Its number goes in the receipt so the registration can be
    /// repaired with the arithmetic on the record; it changes no bar.
    ///
    /// **TWO THINGS THE COMPANION IS NOT, both measured here rather than
    /// argued.** (1) It is NOT the same PERTURBATION SIZE: the shift is
    /// `sign(x)·0.01114·‖row‖₂` applied to EVERY entry, so the row moves by
    /// `√n · 1.114 %` of its own norm — on the 6-coordinate span union that is
    /// `√6 = 2.449×` the registered relative size, not `1.114 %`. (2) It is NOT
    /// comparable with G9(a)'s population: it scores every step (the perturbed
    /// left row is directional almost everywhere), so its scored count is
    /// G9(a)'s scored PLUS G9(a)'s excluded, and
    /// `disagreementsOnUnperturbedExcludedSteps` records how much of its
    /// disagreement count comes from steps G9(a) never scores.
    private func g9Outcome(ctx: ModelContext, perturbationRelative: Double,
                           additiveRelative: Double = 0) throws -> G9Outcome {
        let table = ctx.table
        let rightMuscles = ctx.displayedMuscles.filter { $0.hasSuffix("_r") }.sorted()
        var outcome = G9Outcome()
        let leftMuscles = rightMuscles.map { String($0.dropLast(2)) + "_l" }
        outcome.pairs = rightMuscles.count
        guard rightMuscles.count == 16, Set(leftMuscles).isSubset(of: Set(ctx.displayedMuscles)) else {
            XCTFail("the 16 displayed bilateral pairs did not resolve")
            return outcome
        }

        let rightCoords = spanUnion(rightMuscles, table: table)
        let leftCoords = rightCoords.map { $0.hasSuffix("_r") ? String($0.dropLast(2)) + "_l" : $0 }
        let noise = fixtureNoise(rightCoords.count)
        let neutralIndex = try XCTUnwrap(table.poseIndex("neutral"))
        let base = Self.orderedPose(ctx: ctx, table: table, poseIndex: neutralIndex)

        // Mirrored single-DOF sweeps: the SAME coordinate values and increments
        // applied to the _l coordinate instead of the _r one.
        let sweeps: [(right: String, left: String, from: Double, to: Double)] = [
            ("knee_angle_r", "knee_angle_l", 0, 140),
            ("hip_flexion_r", "hip_flexion_l", -20, 120),
            ("ankle_angle_r", "ankle_angle_l", -30, 20),
        ]
        let step = 10.0 * .pi / 180.0

        for sweep in sweeps {
            let ri = try XCTUnwrap(ctx.dofNames.firstIndex(of: sweep.right))
            let li = try XCTUnwrap(ctx.dofNames.firstIndex(of: sweep.left))
            let from = sweep.from * .pi / 180.0
            let to = sweep.to * .pi / 180.0
            let count = max(2, Int(((to - from) / step).rounded()) + 1)
            for i in 0..<(count - 1) {
                let qa = from + Double(i) * step
                let qb = from + Double(i + 1) * step
                var midRight = base, midLeft = base
                midRight[ri] = 0.5 * (qa + qb)
                midLeft[li] = 0.5 * (qa + qb)

                guard let rowsR = ctx.momentArms(pose: midRight, muscles: rightMuscles,
                                                 coordinates: rightCoords),
                      let rowsL = ctx.momentArms(pose: midLeft, muscles: leftMuscles,
                                                 coordinates: leftCoords) else { continue }
                let dqR = rightCoords.map { $0 == sweep.right ? (qb - qa) : 0.0 }
                let dqL = leftCoords.map { $0 == sweep.left ? (qb - qa) : 0.0 }

                for m in 0..<rightMuscles.count {
                    let rowR = rowsR[m]
                    // The one-sided error class: perturb ONE leg's moment-arm row.
                    var rowL = rowsL[m]
                    if perturbationRelative != 0 {
                        rowL = rowL.map { $0 * (1.0 + perturbationRelative) }
                    }
                    if additiveRelative != 0 {
                        let norm = rowL.reduce(0) { $0 + $1 * $1 }.squareRoot()
                        rowL = rowL.map { $0 - (($0 >= 0 ? 1.0 : -1.0) * additiveRelative * norm) }
                    }
                    let dR = MuscleLengthModeClassifier.stepDeadbandMetres(
                        momentArmRow: rowR, jointNoiseRadians: noise,
                        velocityNoiseGain: Self.velocityGain)
                    let dL = MuscleLengthModeClassifier.stepDeadbandMetres(
                        momentArmRow: rowL, jointNoiseRadians: noise,
                        velocityNoiseGain: Self.velocityGain)
                    let vR = MuscleLengthModeClassifier.lengthRate(momentArmRow: rowR, jointVelocity: dqR)
                    let vL = MuscleLengthModeClassifier.lengthRate(momentArmRow: rowL, jointVelocity: dqL)
                    let modeR = MuscleLengthModeClassifier.classify(value: vR, deadband: dR)
                    let modeL = MuscleLengthModeClassifier.classify(value: vL, deadband: dL)

                    // The SAME step with the left row UNPERTURBED. On the
                    // unperturbed run this is the same array and the same
                    // answer; on a perturbed run it is the only way to say
                    // whether a disagreement landed inside G9(a)'s population
                    // or on a step G9(a) excludes.
                    let unperturbedRowL = rowsL[m]
                    let unperturbedDL = MuscleLengthModeClassifier.stepDeadbandMetres(
                        momentArmRow: unperturbedRowL, jointNoiseRadians: noise,
                        velocityNoiseGain: Self.velocityGain)
                    let unperturbedVL = MuscleLengthModeClassifier.lengthRate(
                        momentArmRow: unperturbedRowL, jointVelocity: dqL)
                    let unperturbedModeL = MuscleLengthModeClassifier.classify(
                        value: unperturbedVL, deadband: unperturbedDL)
                    let scoredUnperturbed = modeR.isDirectional || unperturbedModeL.isDirectional

                    guard modeR.isDirectional || modeL.isDirectional else {
                        outcome.excluded += 1
                        continue
                    }
                    outcome.scored += 1
                    if modeR != modeL {
                        outcome.disagreements += 1
                        if scoredUnperturbed {
                            outcome.disagreementsOnUnperturbedScoredSteps += 1
                        } else {
                            outcome.disagreementsOnUnperturbedExcludedSteps += 1
                        }
                        if outcome.detail.count < 12 {
                            outcome.detail.append("\(rightMuscles[m])@\(sweep.right)"
                                                  + String(format: "[%.3f]", 0.5 * (qa + qb)))
                        }
                    }
                }
            }
        }
        return outcome
    }

    // MARK: - Deadband units

    /// The formula is dimensionally what it claims, its three faces stand in the
    /// registered relationship, and the frozen constants are the frozen values.
    func testTheDeadbandFormulaIsDimensionallyWhatItClaims() throws {
        let arm = [0.05, -0.02]
        let noise = [1.0e-3, 2.0e-3]
        let gain = Self.velocityGain
        let s = MuscleLengthModeClassifier.jitterMetres(momentArmRow: arm, jointNoiseRadians: noise)
        let expectedS = (pow(0.05 * 1.0e-3, 2) + pow(0.02 * 2.0e-3, 2)).squareRoot()
        XCTAssertEqual(s, expectedS, accuracy: 1e-18)

        let step = MuscleLengthModeClassifier.stepDeadbandMetres(
            momentArmRow: arm, jointNoiseRadians: noise, velocityNoiseGain: gain)
        XCTAssertEqual(step, MuscleLengthModeClassifier.k * gain * s, accuracy: 1e-18)

        let dt = 1.0 / 30.0
        let rate = MuscleLengthModeClassifier.rateDeadbandMetresPerSecond(
            momentArmRow: arm, jointNoiseRadians: noise, velocityNoiseGain: gain, sampleInterval: dt)
        XCTAssertEqual(rate, step / dt, accuracy: 1e-15, "D_rate is D_step over one sample interval")

        // The floor binds where the jitter term is smaller than ten storage quanta.
        let floored = MuscleLengthModeClassifier.stepDeadbandMetres(
            momentArmRow: [0], jointNoiseRadians: [0], velocityNoiseGain: gain)
        XCTAssertEqual(floored, MuscleLengthModeClassifier.lengthQuantisationFloorMetres)

        // sqrt(2 c0) is the registered upper bound on witness B's difference noise.
        let filter = WindowedDerivativeFilter(taps: MuscleLengthModeClassifier.taps)
        let c0 = filter.posCoefficients[filter.halfWindow]
        let sumSquares = filter.posCoefficients.reduce(0) { $0 + $1 * $1 }
        XCTAssertEqual(sumSquares, c0, accuracy: 1e-12,
                       "a projection smoother satisfies sum(c_i^2) = c_0")
        let diff = MuscleLengthModeClassifier.differenceDeadbandMetres(
            momentArmRow: arm, jointNoiseRadians: noise, centreCoefficient: c0)
        XCTAssertEqual(diff, MuscleLengthModeClassifier.k * (2 * c0).squareRoot() * s, accuracy: 1e-18)

        // The classification table, including the closed third state.
        XCTAssertEqual(MuscleLengthModeClassifier.classify(value: 2, deadband: 1), .lengthening)
        XCTAssertEqual(MuscleLengthModeClassifier.classify(value: -2, deadband: 1), .shortening)
        XCTAssertEqual(MuscleLengthModeClassifier.classify(value: 1, deadband: 1),
                       .noChangeThisViewCanResolve)
        XCTAssertEqual(MuscleLengthModeClassifier.classify(value: .nan, deadband: 1), .indeterminate)

        // Warmed-frame arithmetic, pinned once and used by every clip-face gate.
        XCTAssertEqual(MuscleLengthModeClassifier.warmedFrameCount(samples: 122), 114)
        XCTAssertEqual(MuscleLengthModeClassifier.firstWarmedIndex(), 4)
        print(String(format: "MODE-METRIC deadband k=%.1f g_vel=%.6f c0=%.9f "
                     + "sqrt2c0=%.6f shrinkage=%.9f floor=%.1e",
                     MuscleLengthModeClassifier.k, gain, c0, (2 * c0).squareRoot(),
                     1.0 / (1.0 - c0).squareRoot(),
                     MuscleLengthModeClassifier.lengthQuantisationFloorMetres))
    }

    // MARK: - Solved-pose fixture provenance and staleness

    /// The staleness guard. A stale fixture is REFUSED, never scored.
    func testTheSolvedPoseFixtureMatchesTheLiveModel() throws {
        let ctx = try context()
        let bundle = Bundle(for: type(of: self))
        let liveSHA = try SolvedPoseFixture.modelSHA256(bundle: bundle)
        for clip in Self.scoredClips {
            let fixture = try SolvedPoseFixture.load(clip: clip, bundle: bundle)
            print("MODE-METRIC fixture clip=\(clip) frames=\(fixture.frames.count) "
                  + "dofs=\(fixture.dofNames.count) markers=\(fixture.markerNames.count) "
                  + "taps=\(fixture.sgTaps) dt_values=\(fixture.distinctIntervals.count) "
                  + String(format: "dt=%.6f span=%.6f", fixture.sampleInterval,
                           fixture.timestamps.last! - fixture.timestamps.first!))
            XCTAssertEqual(fixture.formatId, SolvedPoseFixture.formatId)
            XCTAssertEqual(fixture.modelSHA256, liveSHA, "the fixture is stale against the live model")
            XCTAssertEqual(fixture.dofNames, ctx.dofNames, "DOF name order changed under the fixture")
            XCTAssertEqual(fixture.dofNames.count, ctx.dofNames.count)
            XCTAssertEqual(fixture.sgTaps, MuscleLengthModeClassifier.taps)
            XCTAssertEqual(fixture.frames.count, 122)
            XCTAssertEqual(fixture.distinctIntervals.count, 1,
                           "the clip is not uniformly sampled")
        }
    }

    // MARK: - G2: temporal stability and non-degeneracy

    /// G2(a)+(e). Flicker `<= 1.0 %`, grey-transition `<= 2.0 %`, scored
    /// SEPARATELY per clip.
    ///
    /// ⚠️ **This method is GREEN VACUOUSLY and that is not a satisfied stability
    /// bar.** Both denominators are 0 on both pinned clips because no capsule is
    /// admitted, so `flickerRate` and `greyTransitionRate` return the `0` of an
    /// empty ratio and clear their bars without measuring anything. The vacuity
    /// is asserted and recorded — as vacuity — in
    /// `testG2NonDegeneracyOnThePinnedClips`, which is exactly the degenerate
    /// case clauses (b)–(f) were registered to catch. Do not read this method's
    /// pass as evidence about temporal stability.
    func testG2TemporalStabilityOnThePinnedClips() throws {
        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g2a clip=\(clip) flicker_centres=\(t.flickerCentres) "
                  + "flicker_denominator=\(t.flickerDenominator) "
                  + String(format: "flicker_rate=%.6f", t.flickerRate)
                  + " grey_transitions=\(t.greyTransitions) grey_denominator=\(t.greyDenominator) "
                  + String(format: "grey_rate=%.6f", t.greyTransitionRate))
            XCTAssertLessThanOrEqual(t.flickerRate, 0.01, "G2(a) on \(clip)")
            XCTAssertLessThanOrEqual(t.greyTransitionRate, 0.02, "G2(e) on \(clip)")
        }
    }

    /// G2(b)(c)(d)(f). Non-degeneracy and the power floor. Flicker alone is
    /// passed PERFECTLY by an all-third-state or all-INDETERMINATE output and is
    /// invariant under sign inversion, so without these this gate rewards a layer
    /// that says nothing.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// All four bars are UNCHANGED and still asserted through `RegisteredBar`.
    /// What changed is that the method now RECORDS the empty population instead
    /// of demanding a non-empty one. Pins from
    /// `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log` lines 8126 and 8158
    /// (`grep 'MODE-METRIC g2bcdf'`) plus 8165-8166 (`g2a`): every population is
    /// EXACTLY ZERO on both clips — 0 admitted capsules, 0 defined samples, 0
    /// total samples, no per-capsule entry at all. Bidirectional: the day a
    /// capsule IS admitted these pins go red, which is the intended way to force
    /// a fresh adjudication instead of letting a new population slide in under
    /// an old verdict.
    ///
    /// It also records G2(a)/(e)'s vacuity explicitly: `flicker 0/0` and
    /// `grey 0/0` are EMPTY DENOMINATORS, not satisfied stability bars, and
    /// `testG2TemporalStabilityOnThePinnedClips` passes only because an empty
    /// ratio returns 0.
    /// VERDICT: G2(b), (c), (d) and (f) all FAILED against their registered
    /// bars, and (a)/(e) are vacuous. The layer is NOT SHIPPED. Reopening needs
    /// richer fixtures — a production-grade 20-marker solved-pose clip — plus a
    /// fresh adjudication.
    func testG2NonDegeneracyOnThePinnedClips() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g2DefinedSamples, 300, "G2(f)'s registered floor")
        XCTAssertEqual(RegisteredBar.g2DirectionalFraction, 0.40, "G2(b)'s registered bar")
        XCTAssertEqual(RegisteredBar.g2CapsuleDirectionalFraction, 0.10, "G2(c)'s registered bar")
        XCTAssertEqual(RegisteredBar.g2DefinedFraction, 0.90, "G2(d)'s registered bar")
        XCTAssertEqual(RegisteredBar.g2FlickerRate, 0.01, "G2(a)'s registered bar")
        XCTAssertEqual(RegisteredBar.g2GreyTransitionRate, 0.02, "G2(e)'s registered bar")

        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            let perCapsule = t.perCapsuleDirectionalFraction
                .map { String(format: "%@=%.4f", $0.key, $0.value) }.sorted()
            print("MODE-METRIC g2bcdf clip=\(clip) admitted=\(t.admittedCapsules.count) "
                  + "defined=\(t.definedSamples) total=\(t.totalSamples) "
                  + String(format: "directional=%.6f defined_fraction=%.6f",
                           t.directionalFraction, t.definedFraction)
                  + " per_capsule=[\(perCapsule.joined(separator: ","))]")

            // The MEASURED outcome, pinned to the receipt: exactly zero, not
            // "small". An empty population is the finding.
            XCTAssertEqual(t.admittedCapsules.count, 0,
                           "G2 pin on \(clip): the admitted capsule set is empty")
            XCTAssertEqual(t.definedSamples, 0, "G2(f) pin on \(clip)")
            XCTAssertEqual(t.totalSamples, 0, "G2 pin on \(clip): no capsule sample exists")
            XCTAssertEqual(t.directionalFraction, 0.0, accuracy: 0, "G2(b) pin on \(clip)")
            XCTAssertEqual(t.definedFraction, 0.0, accuracy: 0, "G2(d) pin on \(clip)")
            XCTAssertTrue(t.perCapsuleDirectionalFraction.isEmpty,
                          "G2(c) pin on \(clip): there is no capsule to score")
            XCTAssertEqual(t.minimumCapsuleDirectionalFraction, 0.0, accuracy: 0,
                           "G2(c) pin on \(clip): the minimum over an empty set is the "
                           + "collection's own 0, not a measurement")

            // G2(a)/(e): VACUOUS BY EMPTY DENOMINATOR. Recorded as such here so
            // the sibling method's green cannot be read as a stability result.
            XCTAssertEqual(t.flickerDenominator, 0,
                           "G2(a) on \(clip) is vacuous: the flicker denominator is empty")
            XCTAssertEqual(t.flickerCentres, 0, "G2(a) numerator on \(clip)")
            XCTAssertEqual(t.greyDenominator, 0,
                           "G2(e) on \(clip) is vacuous: the grey denominator is empty")
            XCTAssertEqual(t.greyTransitions, 0, "G2(e) numerator on \(clip)")
            print("MODE-VERDICT gate=G2(a)+G2(e) clip=\(clip) outcome=VACUOUS_EMPTY_DENOMINATOR "
                  + "flicker=\(t.flickerCentres)/\(t.flickerDenominator) "
                  + "grey=\(t.greyTransitions)/\(t.greyDenominator) "
                  + "note=an empty ratio returns 0 and clears the bar without measuring anything; "
                  + "this is NOT a satisfied stability bar")

            // The VERDICT for (b)(c)(d)(f).
            XCTAssertLessThan(t.definedSamples, RegisteredBar.g2DefinedSamples,
                              "G2(f) is recorded as FAILED on \(clip)")
            XCTAssertLessThan(t.directionalFraction, RegisteredBar.g2DirectionalFraction,
                              "G2(b) is recorded as FAILED on \(clip)")
            XCTAssertLessThan(t.minimumCapsuleDirectionalFraction,
                              RegisteredBar.g2CapsuleDirectionalFraction,
                              "G2(c) is recorded as FAILED on \(clip)")
            XCTAssertLessThan(t.definedFraction, RegisteredBar.g2DefinedFraction,
                              "G2(d) is recorded as FAILED on \(clip)")
            recordFailedGate("G2(b)+G2(c)+G2(d)+G2(f)", clip: clip,
                             measured: "admitted=0 defined=0 total=0 directional=0.000000 "
                                       + "defined_fraction=0.000000 per_capsule=[]",
                             bar: "defined >= \(RegisteredBar.g2DefinedSamples), "
                                  + "directional >= \(RegisteredBar.g2DirectionalFraction), "
                                  + "per capsule >= \(RegisteredBar.g2CapsuleDirectionalFraction), "
                                  + "defined fraction >= \(RegisteredBar.g2DefinedFraction)",
                             why: "the fail-closed clip verdict admits no capsule on either "
                                  + "pinned clip, so there is no population to score")
        }
    }

    // MARK: - G7: production-path correctness

    /// G7(a). Two-witness sign agreement `>= 99.0 %` over `>= 500`
    /// jointly-clearing muscle-frames per clip. The floor is HELD at 500: at the
    /// registered minimum admitted set of 4 capsules the ceiling is `4 x 113 =
    /// 452`, so a 4-capsule outcome is an AUTOMATIC under-power FAIL, registered
    /// before any data was seen.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// Both bars are UNCHANGED and still asserted through `RegisteredBar`; the
    /// 500-frame floor is still the registered AUTOMATIC under-power fail. What
    /// changed is that the method now RECORDS the zero population. Pins from
    /// `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log` lines 8248 and 8251
    /// (`grep 'MODE-METRIC g7a'`): 0 jointly-clearing muscle-frames, 0
    /// agreements, 0 rule-3 exclusions, 0 signature changes, ceiling 0, over 114
    /// warmed frames. Bidirectional in the usual way.
    ///
    /// This is also where the DECISION COHERENCE lives, because "the production
    /// path" is the thing being decided about: the admitted set is empty on BOTH
    /// pinned clips AND the layer has ZERO call sites in shipping code, so
    /// "NOT SHIPPED" is machine-checked here rather than asserted in prose. The
    /// scan mirrors G3(i)'s style — comment-stripped source, so a doc comment
    /// naming a symbol is not mistaken for a call.
    /// VERDICT: G7(a) FAILED against its registered bar (under-power). The layer
    /// is NOT SHIPPED. Reopening needs richer fixtures — a production-grade
    /// 20-marker solved-pose clip — plus a fresh adjudication.
    func testG7TwoWitnessAgreementOnTheProductionPath() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g7WitnessJointFrames, 500, "G7(a)'s registered power floor")
        XCTAssertEqual(RegisteredBar.g7WitnessAgreement, 0.99, "G7(a)'s registered agreement bar")

        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            let ceiling = t.admittedMuscles.count * max(0, t.warmedCount - 1)
            print("MODE-METRIC g7a clip=\(clip) joint_clearing=\(t.witnessJointFrames) "
                  + "agree=\(t.witnessAgreements) "
                  + String(format: "agreement=%.6f", t.witnessAgreement)
                  + " rule3_excluded=\(t.rule3Excluded) "
                  + "signature_changes=\(t.signatureChanges) "
                  + "ceiling=\(ceiling)")

            // The MEASURED outcome, pinned to the receipt.
            XCTAssertEqual(t.warmedCount, 114, "G7 pin on \(clip): the warmed-frame count moved")
            XCTAssertEqual(t.admittedMuscles.count, 0, "G7 pin on \(clip): no muscle is admitted")
            XCTAssertEqual(t.witnessJointFrames, 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.witnessAgreements, 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.witnessAgreement, 0.0, accuracy: 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.rule3Excluded, 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.signatureChanges, 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(ceiling, 0, "G7(a) pin on \(clip): the ceiling is 0 admitted x 113")

            // DECISION COHERENCE, half one: the admitted set is empty on BOTH
            // pinned clips, which is the reason the gate has no population.
            XCTAssertEqual(t.admittedCapsules.count, 0,
                           "coherence: a capsule is admitted on \(clip) while the recorded verdict "
                           + "says the mask admits nothing")

            // The VERDICT.
            XCTAssertLessThan(t.witnessJointFrames, RegisteredBar.g7WitnessJointFrames,
                "G7(a) is recorded as FAILED on \(clip): under-power, 0 jointly-clearing frames")
            recordFailedGate("G7(a)", clip: clip,
                             measured: "joint_clearing=0 agree=0 agreement=0.000000 ceiling=0 "
                                       + "over \(t.warmedCount) warmed frames",
                             bar: ">= \(RegisteredBar.g7WitnessJointFrames) jointly-clearing "
                                  + "muscle-frames at >= \(RegisteredBar.g7WitnessAgreement) "
                                  + "agreement",
                             why: "no admitted muscle, so witness A and witness B never both clear")
        }

        // DECISION COHERENCE, half two: NOT SHIPPED, checked against the tree.
        let census = try Self.lengthModeShippingSymbolCensus()
        print("MODE-VERDICT coherence layer=NOT_SHIPPED app_side_call_sites=0 "
              + "symbol_census=[" + census.map { "\($0.key)->\($0.value.sorted().joined(separator: "+"))" }
                  .sorted().joined(separator: " ") + "]")
        XCTAssertEqual(census, Self.lengthModeDeclaringFiles,
            "coherence: the length-mode layer is reachable from shipping code. Every one of its "
            + "symbols must appear ONLY in the files that declare or define it — a new file in "
            + "this census is a call site, and a call site means the layer ships, which "
            + "contradicts the recorded FAIL verdict.")
    }

    /// G7(b) STALE-POSE SENTINEL. A constant length trace is not a small error,
    /// it is the layer not being connected to the pose.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// Both bars are UNCHANGED and still asserted through `RegisteredBar`. What
    /// changed is that the method now RECORDS that the sentinel COULD NOT RUN.
    /// Pins from `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log` lines 8240
    /// and 8243 (`grep 'MODE-METRIC g7b'`): 0 admitted muscles,
    /// `min_length_range = 0.000000000e+00`, `reimpose_max_delta = 0.000e+00`.
    ///
    /// ⚠️ The re-impose clause is VACUOUS, not passed: `buildTraversal` returns
    /// early when the admitted set is empty (`SolvedPoseFixture.swift:373-377`),
    /// so the re-impose block never executes and `0.000e+00` is an untouched
    /// initial value rather than a measurement. Recorded as vacuity here.
    /// Bidirectional in the usual way.
    /// VERDICT: G7(b) FAILED against its registered bar. The layer is NOT
    /// SHIPPED. Reopening needs richer fixtures — a production-grade 20-marker
    /// solved-pose clip — plus a fresh adjudication.
    func testG7StalePoseSentinel() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g7MinimumLengthRangeMetres, 1.0e-6, "G7(b)'s registered range")
        XCTAssertEqual(RegisteredBar.g7ReimposedPoseDeltaMetres, 1.0e-9, "G7(b)'s registered delta")

        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g7b clip=\(clip) admitted_muscles=\(t.admittedMuscles.count) "
                  + String(format: "min_length_range=%.9e reimpose_max_delta=%.3e",
                           t.minimumLengthRange, t.reimposedPoseMaxDelta))

            // The MEASURED outcome, pinned to the receipt.
            XCTAssertEqual(t.admittedMuscles.count, 0, "G7(b) pin on \(clip)")
            XCTAssertEqual(t.minimumLengthRange, 0.0, accuracy: 0,
                           "G7(b) pin on \(clip): no admitted trace exists to have a range")
            XCTAssertEqual(t.reimposedPoseMaxDelta, 0.0, accuracy: 0,
                           "G7(b) pin on \(clip): the re-impose probe never ran")

            // The VERDICT.
            XCTAssertLessThan(t.admittedMuscles.count, 1,
                "G7(b) is recorded as FAILED on \(clip): the sentinel cannot run without an "
                + "admitted muscle")
            XCTAssertLessThanOrEqual(t.minimumLengthRange, RegisteredBar.g7MinimumLengthRangeMetres,
                "G7(b) is recorded as FAILED on \(clip): the registered clause needs a range "
                + "STRICTLY GREATER than 1e-6 m and there is no trace at all")
            print("MODE-VERDICT gate=G7(b)-reimpose clip=\(clip) outcome=VACUOUS_UNREACHABLE "
                  + "reimpose_max_delta=\(t.reimposedPoseMaxDelta) "
                  + "note=buildTraversal returns before the re-impose probe when the admitted set "
                  + "is empty, so this 0 is an initial value and NOT a satisfied bar")
            recordFailedGate("G7(b)", clip: clip,
                             measured: "admitted_muscles=0 min_length_range=0.000000000e+00 "
                                       + "reimpose_max_delta=0.000e+00 (vacuous)",
                             bar: "range > \(RegisteredBar.g7MinimumLengthRangeMetres) m, "
                                  + "re-impose <= \(RegisteredBar.g7ReimposedPoseDeltaMetres) m",
                             why: "no admitted muscle, so the stale-pose sentinel has no trace "
                                  + "to inspect")
        }
    }

    // MARK: - G8: secular-drift screen

    /// G8(a). END-TO-END OLS trend excursion over the WARMED frames, as a
    /// fraction of that same warmed trace's peak-to-peak range: `<= 30 %` for
    /// every admitted muscle, and `<= 20 %` for at least 90 % of them.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// All three bars are UNCHANGED and still asserted through `RegisteredBar`.
    /// What changed is that the method now RECORDS that the screen SCREENED
    /// NOTHING. Pins from `/tmp/biomotion-tests.BoBjif/subset/xcodebuild.log`
    /// lines 8263 and 8267 (`grep 'MODE-METRIC g8a'`): 0 muscles scored,
    /// `worst = 0.000000` (the empty-collection fallback, not a measured
    /// excursion), 0 over the 20 % secondary. Bidirectional in the usual way.
    ///
    /// ⚠️ The 30 % and 20 % clauses are VACUOUS here for the same reason G2(a)
    /// is: there is no trace to have a trend. Recorded as vacuity, never as a
    /// clean drift screen.
    /// VERDICT: G8(a) FAILED against its registered population requirement. The
    /// layer is NOT SHIPPED. Reopening needs richer fixtures — a
    /// production-grade 20-marker solved-pose clip — plus a fresh adjudication.
    func testG8SecularDriftScreen() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g8PrimaryExcursion, 0.30, "G8(a)'s registered primary bar")
        XCTAssertEqual(RegisteredBar.g8SecondaryExcursion, 0.20, "G8(a)'s registered secondary bar")
        XCTAssertEqual(RegisteredBar.g8SecondaryFraction, 0.10, "G8(a)'s registered 10 % allowance")

        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            let sorted = t.trendExcursions.sorted { $0.value > $1.value }
            let over = t.trendExcursions.values
                .filter { $0 > RegisteredBar.g8SecondaryExcursion }.count
            print("MODE-METRIC g8a clip=\(clip) muscles=\(t.trendExcursions.count) "
                  + String(format: "worst=%.6f", sorted.first?.value ?? 0)
                  + " over20=\(over) "
                  + "detail=[" + sorted.prefix(8)
                      .map { String(format: "%@=%.4f", $0.key, $0.value) }
                      .joined(separator: ",") + "]")

            // The MEASURED outcome, pinned to the receipt.
            XCTAssertTrue(t.trendExcursions.isEmpty,
                          "G8(a) pin on \(clip): the screen has no population")
            XCTAssertEqual(t.trendExcursions.count, 0, "G8(a) pin on \(clip)")
            XCTAssertEqual(sorted.first?.value ?? 0, 0.0, accuracy: 0,
                           "G8(a) pin on \(clip): `worst` is the empty fallback, not an excursion")
            XCTAssertEqual(over, 0, "G8(a) pin on \(clip)")

            // The VERDICT.
            XCTAssertLessThan(t.trendExcursions.count, 1,
                "G8(a) is recorded as FAILED on \(clip): the registered clause needs at least one "
                + "scored muscle and there are none")
            print("MODE-VERDICT gate=G8(a)-excursions clip=\(clip) outcome=VACUOUS_EMPTY_POPULATION "
                  + "worst=0.000000 over20=0 "
                  + "note=the 30 %/20 % clauses cannot be read as a clean drift screen; there is "
                  + "no admitted trace to have a trend")
            recordFailedGate("G8(a)", clip: clip,
                             measured: "muscles=0 worst=0.000000 over20=0 detail=[]",
                             bar: "at least one scored muscle, all <= "
                                  + "\(RegisteredBar.g8PrimaryExcursion), at most "
                                  + "\(RegisteredBar.g8SecondaryFraction) of them over "
                                  + "\(RegisteredBar.g8SecondaryExcursion)",
                             why: "no admitted muscle, so no L_MT trace exists to screen for drift")
        }
    }

    /// G8(b). REPORTED, not gated: the apparent frame-to-frame change in the two
    /// anatomically rigid distances these 5-joint clips can actually supply. The
    /// StaticHoldDetector construction applied to the only rigid pairs available
    /// — ten of its ten pairs are unformable from PELVIS/LAJC/RAJC/LTOE/RTOE, and
    /// running the same measure on IK OUTPUT would be circular.
    func testG8FootSegmentNoiseReceipt() throws {
        let bundle = Bundle(for: type(of: self))
        for clip in Self.scoredClips {
            let frames = try GaitClipFixture.load(clip, bundle: bundle).frames
            for (a, b, label) in [("left_foot_joint", "left_toes_joint", "left_foot"),
                                  ("right_foot_joint", "right_toes_joint", "right_foot")] {
                var distances: [Double] = []
                var times: [Double] = []
                for frame in frames {
                    guard let pa = frame.joints.first(where: { $0.id == a })?.worldPosition,
                          let pb = frame.joints.first(where: { $0.id == b })?.worldPosition
                    else { continue }
                    distances.append(Double(simd_distance(pa, pb)))
                    times.append(frame.timestamp)
                }
                guard distances.count > 1 else { continue }
                var deltas: [Double] = []
                var speeds: [Double] = []
                for i in 1..<distances.count {
                    let dt = times[i] - times[i - 1]
                    let d = abs(distances[i] - distances[i - 1])
                    deltas.append(d)
                    if dt > 0 { speeds.append(d / dt) }
                }
                print(String(format: "MODE-METRIC g8b clip=%@ pair=%@ n=%d "
                             + "median_delta_m=%.9f p95_delta_m=%.9f median_speed_mps=%.9f",
                             clip, label, deltas.count,
                             MuscleLengthModeClassifier.median(deltas),
                             deltas.sorted()[min(deltas.count - 1, Int(0.95 * Double(deltas.count)))],
                             MuscleLengthModeClassifier.median(speeds)))
                XCTAssertGreaterThan(deltas.count, 0)
            }
        }
    }

    // MARK: - G5: perf and lane receipt

    /// G5. No time bar. A receipt that does not say WHICH set and WHICH stencil
    /// form was measured is the failure.
    func testG5PerFrameCostAndLaneReceipt() throws {
        let ctx = try context()
        var perFrameMode: [Double] = []
        var perFrameJacobian: [Double] = []
        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: ctx)
            perFrameMode.append(t.msPerFrameMode)
            perFrameJacobian.append(t.msPerFrameIdentifiability)
            print(String(format: "MODE-METRIC g5 clip=%@ measured_set=displayed displayed=%d "
                         + "admitted_muscles=%d coordinates=%d stencil=coordinate-block "
                         + "ms_per_frame_mode=%.3f ms_per_frame_identifiability=%.3f "
                         + "traversal_wall_s=%.3f build=Debug host=iOS-Simulator",
                         clip, ctx.displayedMuscles.count, t.admittedMuscles.count,
                         t.stencilCoordinates.count, t.msPerFrameMode,
                         t.msPerFrameIdentifiability, t.traversalSeconds))
            XCTAssertGreaterThan(t.msPerFrameMode, 0)
            XCTAssertGreaterThan(t.msPerFrameIdentifiability, 0)
        }
        // The structural ratio the subset stencil is motivated by, recomputed
        // rather than quoted: 520 x 169 against the displayed block.
        let full = Double(ctx.muscleNames.count * ctx.dofNames.count)
        let block = Double(ctx.displayedMuscles.count * 12)
        print(String(format: "MODE-METRIC g5 full_pairs=%.0f block_pairs=%.0f ratio=%.1f",
                     full, block, full / block))
        XCTAssertEqual(perFrameMode.count, Self.scoredClips.count)
        XCTAssertEqual(perFrameJacobian.count, Self.scoredClips.count)
    }

    // MARK: - Shared clip traversal

    /// ONE cached traversal per clip, shared by G2, G3(iv), G5, G7 and G8. Full
    /// frame, no subsampling: subsampling would change the flicker denominator,
    /// i.e. it is a gate-relevant lever and is frozen out.
    final class ClipTraversal {
        var warmedCount = 0
        var stencilCoordinates: [String] = []
        var admittedCapsules: Set<String> = []
        var admittedMuscles: [String] = []
        var hipCapsules: [String] = []
        var suppressedHipCapsules: [String] = []
        var clipUnidentifiedLowerLimb: Set<String> = []
        var framesWithEmptyUnidentifiedLowerLimb = 0

        var flickerCentres = 0
        var flickerDenominator = 0
        var greyTransitions = 0
        var greyDenominator = 0
        var definedSamples = 0
        var totalSamples = 0
        var directionalSamples = 0
        var perCapsuleDirectionalFraction: [String: Double] = [:]

        var witnessJointFrames = 0
        var witnessAgreements = 0
        var rule3Excluded = 0
        var signatureChanges = 0
        var minimumLengthRange = Double.infinity
        var reimposedPoseMaxDelta = 0.0
        var trendExcursions: [String: Double] = [:]

        var msPerFrameMode = 0.0
        var msPerFrameIdentifiability = 0.0
        var traversalSeconds = 0.0

        var flickerRate: Double { flickerDenominator == 0 ? 0 : Double(flickerCentres) / Double(flickerDenominator) }
        var greyTransitionRate: Double { greyDenominator == 0 ? 0 : Double(greyTransitions) / Double(greyDenominator) }
        var definedFraction: Double { totalSamples == 0 ? 0 : Double(definedSamples) / Double(totalSamples) }
        var directionalFraction: Double { definedSamples == 0 ? 0 : Double(directionalSamples) / Double(definedSamples) }
        var witnessAgreement: Double { witnessJointFrames == 0 ? 0 : Double(witnessAgreements) / Double(witnessJointFrames) }
        var minimumCapsuleDirectionalFraction: Double {
            perCapsuleDirectionalFraction.values.min() ?? 0
        }
    }

    private static var traversalCache: [String: ClipTraversal] = [:]
    private static let traversalLock = NSLock()

    static func traversal(clip: String, context ctx: ModelContext) throws -> ClipTraversal {
        traversalLock.lock()
        defer { traversalLock.unlock() }
        if let cached = traversalCache[clip] { return cached }
        let built = try buildTraversal(clip: clip, ctx: ctx)
        traversalCache[clip] = built
        return built
    }

    // MARK: - Small shared utilities

    static func repositoryRoot() -> URL {
        // This file lives at <root>/BioMotionTests/MuscleLengthModeTests.swift.
        URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent()
    }

    /// Strips `//` line comments and `/* */` block comments. Load-bearing, not
    /// cosmetic: `NimbleEngine.swift` carries the cited measured-decision receipt
    /// inside a comment, and that receipt must NOT be deleted to satisfy a
    /// negative assertion.
    static func strippingComments(_ source: String) -> String {
        var out = ""
        var index = source.startIndex
        var inLine = false, inBlock = false, inString = false
        while index < source.endIndex {
            let c = source[index]
            let next = source.index(after: index)
            let n: Character? = next < source.endIndex ? source[next] : nil
            if inLine {
                if c == "\n" { inLine = false; out.append(c) }
            } else if inBlock {
                if c == "*", n == "/" { inBlock = false; index = next }
            } else if inString {
                if c == "\\" { index = next } else if c == "\"" { inString = false; out.append(c) }
                else { out.append(c) }
            } else if c == "/", n == "/" {
                inLine = true; index = next
            } else if c == "/", n == "*" {
                inBlock = true; index = next
            } else {
                if c == "\"" { inString = true }
                out.append(c)
            }
            index = source.index(after: index)
        }
        return out
    }

    // MARK: - "NOT SHIPPED", checked against the tree

    /// The length-mode layer's symbols, and the ONLY shipping files allowed to
    /// contain each of them: the file that declares it, the file that defines
    /// it, and — for `MuscleLengthMode` and `velocityNoiseGain` — the sibling
    /// file inside the layer that consumes it. Anything else in the census is a
    /// CALL SITE, i.e. the layer being wired into the app, which contradicts the
    /// recorded FAIL verdict and must re-open the adjudication rather than pass
    /// quietly.
    ///
    /// Registered as an exact set on 2026-08-14, from the tree, alongside the
    /// conversion. It goes red in EITHER direction: a new consumer file, or the
    /// disappearance of a declaring file.
    static let lengthModeDeclaringFiles: [String: Set<String>] = [
        "MuscleLengthMode": ["BioMotion/Muscle/MuscleLengthMode.swift",
                             "BioMotion/Muscle/MuscleObservabilityMask.swift"],
        "MuscleObservabilityMask": ["BioMotion/Muscle/MuscleObservabilityMask.swift"],
        "markerPositionJacobianForMarkerNames": ["BioMotion/Nimble/NimbleBridge.h",
                                                 "BioMotion/Nimble/NimbleBridge.mm"],
        "muscleLengthsForIndices": ["BioMotion/Muscle/MomentArmComputer.h",
                                    "BioMotion/Muscle/MomentArmComputer.mm"],
        "velocityNoiseGain": ["BioMotion/Muscle/MuscleLengthMode.swift",
                              "BioMotion/Nimble/NimbleEngine.swift"],
    ]

    /// Which shipping files mention each length-mode symbol, in COMMENT-STRIPPED
    /// source. Stripping is load-bearing exactly as it is in G3(i): a doc
    /// comment naming a symbol is prose, not a call, and the registration's own
    /// text quotes several of these names.
    static func lengthModeShippingSymbolCensus() throws -> [String: Set<String>] {
        let root = repositoryRoot()
        let shipping = root.appendingPathComponent("BioMotion")
        let sourceExtensions: Set<String> = ["swift", "h", "m", "mm", "c", "cpp"]
        let symbols = lengthModeDeclaringFiles.keys.sorted()
        var census: [String: Set<String>] = [:]
        for symbol in symbols { census[symbol] = [] }

        let enumerator = FileManager.default.enumerator(at: shipping,
                                                        includingPropertiesForKeys: nil)
        while let url = enumerator?.nextObject() as? URL {
            guard sourceExtensions.contains(url.pathExtension) else { continue }
            guard let raw = try? String(contentsOf: url, encoding: .utf8) else { continue }
            guard symbols.contains(where: { raw.contains($0) }) else { continue }
            let relative = url.path.replacingOccurrences(of: root.path + "/", with: "")
            let stripped = strippingComments(raw)
            for symbol in symbols where stripped.contains(symbol) {
                census[symbol]?.insert(relative)
            }
        }
        return census
    }

    /// The fixture's pose values re-ordered into the LIVE model's DOF order.
    static func orderedPose(ctx: ModelContext, table: OpenSimReferenceFixture.Table,
                            poseIndex: Int) -> [Double] {
        let values = table.coordinateValues(poseIndex: poseIndex)
        return ctx.dofNames.map { values[$0] ?? 0 }
    }

    /// `nullFraction` per coordinate at whatever pose the shared skeleton holds.
    static func nullFractions(ctx: ModelContext, markers: [String]) -> [Double]? {
        guard let flat = ctx.bridge.markerPositionJacobian(forMarkerNames: markers) else { return nil }
        let rows = 3 * markers.count
        let columns = ctx.dofNames.count
        guard flat.count == rows * columns else { return nil }
        return MuscleObservabilityMask.nullFractions(
            jacobianRowMajor: flat.map(\.doubleValue), rows: rows, columns: columns)
    }

    static func clipMarkerNames(clip: String, context ctx: ModelContext) throws -> [String] {
        let fixture = try SolvedPoseFixture.load(clip: clip,
                                                 bundle: Bundle(for: MuscleLengthModeTests.self))
        return fixture.markerNames
    }
}
