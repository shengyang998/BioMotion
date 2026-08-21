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

    /// G2's MEASURED-OUTCOME pins, transitioned 2026-08-14 (fifteenth round,
    /// person-box sidecar amendment) from the 5-marker lineage's all-zero
    /// population to the 20-marker video-driven fixtures' real one. Receipt:
    /// `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g2bcdf'`.
    ///
    /// Bidirectional in BOTH senses, exactly as the 2026-08-13 conversion
    /// declared: a figure that drifts and a figure that improves past its pin
    /// both turn this file red, because either means the mask, the fixtures, the
    /// classifier or the model moved and the verdict has to be re-adjudicated
    /// rather than silently re-baselined.
    ///
    /// PROVENANCE: these fixtures carry a **macOS-Vision INTERIM** person box
    /// (`bbox_source macos_vision INTERIM`). Nothing pinned here is device-grade.
    ///
    /// ─── RE-PINNED 2026-08-21, sixteenth round, 8.0 s window ───
    /// `FrameSource.analysisWindowSeconds` 4.0 → 8.0 moved retained 120 → 240 and
    /// warmed 112 → 232 on BOTH clips, and every number below moved with it. The
    /// 112-warmed pins are SUPERSEDED-NOT-ERASED: they are the dated receipt of
    /// what the 4 s window measured, and the round-16 header in STATUS.md carries
    /// them beside these. NO BAR MOVED. The window change was pre-registered in
    /// that header BEFORE this regeneration ran, together with the prediction
    /// that all twelve fixture-fixable clauses would be adjudicated in ONE
    /// verdict and that clauses passing at 4 s might fail at 8 s.
    /// WHAT THAT PREDICTION GOT WRONG, recorded because it was stated first:
    /// (1) the G7(a) clearing rate was ASSUMED frame-independent; it was not
    /// (15.82 % → 17.20 % on video_012, 11.20 % → 10.36 % on video_015), so the
    /// projected 658/828 came in at 715/766 — right side of the floor, wrong
    /// arithmetic. (2) G2(a)/(e) rates were predicted "roughly unchanged"; the
    /// video_012 flicker rate FELL 48 % (4.8255 % → 2.5213 %), which is the
    /// single most surprising number in this round and is unexplained.
    /// WHAT GOT WORSE, recorded in the same breath: G2(c) on video_012 fell from
    /// 0.0089 to EXACTLY 0.0 (`gaslat_l` and `gasmed_l` are directional on zero
    /// of 231 transitions), and G8(a) on video_012 rose 0.2971 → 0.4174, which
    /// crosses the 30 % PRIMARY bar it used to clear.
    static let g2Pins: [String: (admitted: Int, defined: Int, total: Int,
                                 directional: Double, definedFraction: Double,
                                 minimumCapsule: Double, weakestCapsule: String)] = [
        "video_012": (14, 3012, 3248, 0.4584993359893758, 0.9273399014778325,
                      0.0, "gaslat_l"),
        "video_015": (24, 5322, 5568, 0.5227358136039083, 0.9558189655172413,
                      0.19594594594594594, "glmed1_r"),
    ]

    /// G7/G8's MEASURED-OUTCOME pins, transitioned 2026-08-14 in the same round
    /// and from the same receipt as `g2Pins`.
    static let g7Pins: [String: (admittedMuscles: Int, admittedCapsules: Int, jointFrames: Int,
                                 agreements: Int, signatureChanges: Int, ceiling: Int,
                                 minimumLengthRange: Double)] = [
        "video_012": (18, 14, 715, 715, 299, 4158, 0.017025392767722636),
        "video_015": (32, 24, 766, 766, 457, 7392, 0.024271537816931255),
    ]

    /// G8(a)'s MEASURED-OUTCOME pins. `over20` is the count of scored muscles
    /// whose trend excursion exceeds the 20 % SECONDARY bar; the registered
    /// allowance is 10 % of the scored set.
    static let g8Pins: [String: (muscles: Int, worst: Double, worstMuscle: String, over20: Int)] = [
        "video_012": (18, 0.417378214824435, "psoas_l", 7),
        "video_015": (32, 0.12018028131218193, "glmax3_l", 0),
    ]

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
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ───
    /// **(iv-a) still PASSES and (iv-b) now FAILS, and that split is the
    /// finding.** Under the 20-marker video-driven fixtures the drive-aware mask
    /// still fires at every warmed frame — `empty_frames = 0` on both clips, so
    /// the mask has never fired on nothing — but the hip block is no longer
    /// suppressed: **6 of 12** hip capsules survive on `video_012` and **12 of
    /// 12** survive on `video_015` (`hip_suppressed = 6` and `0`).
    ///
    /// **This is a REGISTRATION-CONDITIONAL clause, not a repairable defect, and
    /// it is recorded as a FAIL rather than reinterpreted.** (iv-b) was written
    /// against the 5-marker drive, where `hips_joint` never moved and the
    /// fail-closed verdict suppressed the whole hip block by construction. With
    /// twenty markers the hip coordinates ARE identified, so admitting them is
    /// the mask behaving correctly and the clause failing. Reopening it requires
    /// a SUCCESSOR preregistration in the same class as next-steps 42 and 43 —
    /// a corrected instrument, RED-first, with the old clause named
    /// SUPERSEDED-NOT-ERASED. Nothing here weakens it.
    ///
    /// Receipt: `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g3iv'`. PROVENANCE: macOS Vision, INTERIM.
    ///
    /// ─── SUCCESSOR PRE-REGISTRATION, written 2026-08-21, RED-FIRST, BEFORE the
    /// measurement that adjudicates it ───
    ///
    /// **G3(iv-b) stands FAILED PERMANENTLY and is SUPERSEDED-NOT-ERASED BY
    /// G3(iv-b2).** The text above is not edited and the pins are not
    /// re-baselined. Recorded permanently, because it changes how every future
    /// round must read this gate: **(iv-b) CAN NEVER PASS AGAIN UNDER A
    /// HIP-IDENTIFYING DRIVE.** It was registered against a 5-marker drive in
    /// which `hips_joint` had range 0.000000000 on all three axes, so Rule 2
    /// (unmeasured) and Rule 1 (fail-closed) suppressed the whole hip block BY
    /// CONSTRUCTION and the clause was satisfied on a population that could not
    /// have done anything else. With twenty markers the hip coordinates ARE
    /// identified, and admitting them is the mask BEHAVING CORRECTLY. The ONLY
    /// way to make (iv-b) pass again is to delete the hip markers and return to
    /// the vacuous 5-marker populations this round exists to escape — which is
    /// why it is not a repairable gate, must never be reported as passing, and
    /// stands FAILED for good. What it genuinely guarded is not "hips are
    /// suppressed" but "THE MASK NEVER COLOURS A CAPSULE WHOSE MOTION THE DRIVE
    /// CANNOT RESOLVE"; that invariant is what the successor carries, in a form
    /// that survives a change of drive.
    ///
    /// **G3(iv-b2) HIP ADMISSION IS EARNED, PER FRAME, AND EVERY SUPPRESSION
    /// NAMES ITS WITNESS.**
    ///
    /// (b2-i) EVIDENCE FLOOR — closed form, INDEPENDENT CODE PATH. For every
    /// hip-spanning capsule ADMITTED on a clip, for every hip coordinate `j` it
    /// spans, at EVERY warmed frame, the marker-Jacobian column norm
    /// `‖J(q)·eⱼ‖₂` — summed straight down ONE COLUMN of the same
    /// `markerPositionJacobian(forMarkerNames:)` bytes the mask consumed — must
    /// satisfy
    ///     ‖J(q)·eⱼ‖₂  ≥  √0.75 · sigmaVisible  =  8.660254037844386e-3 m/rad.
    /// THIS INTRODUCES NO NEW CONSTANT AND IS NOT A TAUTOLOGY. It is IMPLIED by
    /// Rule 1 and re-derived here from the shipped source rather than quoted:
    /// `MuscleObservabilityMask.nullFractions` (MOM:286-322 — every `MOM:` here is a
    /// line in `BioMotion/Muscle/MuscleObservabilityMask.swift`) accumulates
    /// `retainedⱼ = Σ_{λᵢ ≥ sigmaVisible²} vᵢ[j]²` with `vᵢ = Jᵀuᵢ/σᵢ` and
    /// returns `nullFractionⱼ = √(clamp(1 − retainedⱼ, 0, 1))`. IDENTIFIED means
    /// `nullFractionⱼ ≤ identifiedNullFractionCeiling` (MOM:111) `=
    /// PostureFindings.depthSuppressionFraction = 0.5`
    /// (`PostureFindings.swift:75`), hence `retainedⱼ ≥ 0.75`. Since
    /// `vᵢ[j] = (uᵢ · J·eⱼ)/σᵢ` and `{uᵢ}` is orthonormal, Bessel gives
    /// `retainedⱼ ≤ ‖J·eⱼ‖² / sigmaVisible²`, so identified ⇒
    /// `‖J·eⱼ‖ ≥ √0.75 · sigmaVisible`. `sigmaVisible` stays 1.0e-2 m/rad (MOM:115)
    /// and the crossover stays 0.5; `√0.75 = 0.8660254037844386` is their
    /// CONSEQUENCE, not a lever, and it may not be tuned. Sanity check that the
    /// floor is not itself over-strict: 8.660254e-3 m/rad sits BELOW
    /// `mustNotMaskColumnNormMetresPerRadian = 0.0343` (MOM:119), the measured
    /// column norm of the coordinate this repo proved must NOT be masked.
    /// IT IS A NECESSARY CONDITION, NOT A SUFFICIENT ONE, and that is exactly
    /// what makes it a DIFFERENTIAL test of two computations rather than a
    /// re-execution of the mask: the column norm never touches the Gram/Jacobi
    /// route. It goes RED on an eigenvector-indexing defect, a bad `retained`
    /// accumulation, a mis-applied [0,1] clamp, or on Rule 1's own registered
    /// defect (b) — "it used the 20-marker set when the analysed clip supplies a
    /// different one" — because the columns are taken from the CLIP's own
    /// `fixture.markerNames`. That is precisely the silent over-admission the
    /// superseded clause used to make impossible by accident.
    /// NON-VACUOUS TODAY, checked before registering: the admitted hip-capsule
    /// population is 6 on `video_012` and 12 on `video_015` (12 hip capsules
    /// minus the 6/0 suppressed pinned at :755-756), so neither clip scores this
    /// arm on an empty set. If a future drive admits none, the arm must print
    /// VACUOUS-BY-CONSTRUCTION and must not be counted as a pass.
    ///
    /// (b2-ii) FAIL-CLOSED PERSISTENCE AND A NAMED WITNESS. Suppression stays
    /// FRAME-EXHAUSTIVE: a hip coordinate counts as identified for the clip only
    /// if it is identified at EVERY warmed frame, and no "identified on ≥ X % of
    /// frames" lever may be introduced by this or any successor. Additionally,
    /// every SUPPRESSED hip capsule must be ATTRIBUTABLE: the gate prints, per
    /// suppressed capsule, the SPECIFIC coordinate that suppressed it plus the
    /// worst warmed-frame index and that frame's `nullFraction`, and asserts that
    /// the suppressed-hip set is EXACTLY the set of hip capsules spanning
    /// `clipUnidentifiedLowerLimb ∪ unmeasured`.
    /// NOT A TAUTOLOGY, and the reason is a measured asymmetry in the shipped
    /// code rather than an argument: `MuscleObservabilityMask.isSuppressed`
    /// (MOM:430-433) reads the FULL `identified` set over all model coordinates,
    /// while `clipUnidentifiedLowerLimb` is restricted to the model's LEADING
    /// 20-coordinate lower-limb block (`SolvedPoseFixture.swift:247-268`). A hip
    /// capsule suppressed by a coordinate OUTSIDE that block therefore turns this
    /// clause RED. REGISTERED CONSEQUENCE, before the run: that red is
    /// ADJUDICATED, never absorbed and never repaired by widening the witness
    /// set — it would mean the hip block is being greyed from outside the lower
    /// limb, which is a finding.
    /// The present asymmetry — `video_012` suppressing 6 of 12 while `video_015`
    /// suppresses 0 of 12, on the SAME model and the SAME pipeline (:755-756) —
    /// must resolve to a NAMED coordinate or this clause FAILS. A mask that greys
    /// half a block for no nameable reason is not a derivation, and this is the
    /// clause that catches it.
    ///
    /// (b2-iii) ANTI-VACUITY, INVERTED ACCEPTANCE PRESERVED. On the same
    /// 20-marker drive and the SAME warmed frames, every one of the 72
    /// coordinates the 20-marker Jacobian leaves identically zero — the same list
    /// G3(ii) already asserts against — must be UNIDENTIFIED at EVERY warmed
    /// frame. G3(ii) asserts this at ONE static `neutral` pose using the
    /// `JointMapping.primary` marker list; this extends it to the CLIP'S OWN
    /// poses and the CLIP'S OWN marker list, which is where the mask actually
    /// runs. That population is 72 and non-empty, so this arm is a real
    /// measurement. Its companion arm — "every capsule spanning one of those 72
    /// must remain suppressed" — may be scored on an EMPTY population, and that
    /// possibility is declared rather than discovered: all 72 are trunk and
    /// rib-cage coordinates (`Abs_*` and `T*_r*_{X,Y,Z}`, re-read from the list on
    /// 2026-08-21), NOT ONE of them is a hip, knee or ankle coordinate, and the
    /// only declared capsules that reach the trunk at all are `ercspn_r/_l` and
    /// `psoas_r/_l`. Whether any of those four SPANS one of the 72 is deliberately
    /// NOT asserted here, because it has not been measured; the arm must PRINT its
    /// population size, and if that size is zero it must print
    /// VACUOUS-BY-CONSTRUCTION and MUST NOT be counted as a pass.
    /// `framesWithEmptyUnidentifiedLowerLimb == 0` from G3(iv-a) is retained
    /// UNCHANGED, and "fires on 0" remains DISPROOF.
    ///
    /// (b2-iv) BIDIRECTIONAL ADMISSION CENSUS. `hip_suppressed` and `admitted`
    /// stay pinned per clip to the receipt they are read from (`video_012`: 6/12
    /// suppressed, 14 admitted, 112 warmed; `video_015`: 0/12 suppressed, 24
    /// admitted, 112 warmed — :755-756) in BOTH directions: a census that DRIFTS
    /// and a census that IMPROVES both turn this RED and force a fresh
    /// adjudication instead of a silent re-baseline. PROVENANCE, binding: these
    /// fixtures carry `bbox_source macos_vision INTERIM`, so the census pins are
    /// provisional ON THE FIXTURES and must be RE-PINNED — not re-interpreted —
    /// when device-grade fixtures land.
    ///
    /// ⚠️ **EXPECTED VERDICTS, DECLARED BEFORE THE RUN (adversarial review,
    /// 2026-08-21), so that no arm of this clause is quoted as evidence it cannot
    /// supply:** (b2-i) is a THEOREM under the shipped mask, not an open
    /// measurement of hip admissibility — `clipIdentifiedCoordinates`
    /// (MOM:387-399) is a fail-closed AND over every warmed frame and
    /// `isIdentified` is `nullFraction <= 0.5` (MOM:383-385), so ADMITTED already
    /// forces retained ≥ 0.75 and hence the floor. Its only reachable failure
    /// modes are an eigen-route implementation defect and a marker-set mismatch;
    /// it is EXPECTED TO PASS and its value is as an independent-code-path guard
    /// against exactly those two. (b2-iv) re-pins numbers already asserted at
    /// :755-756 and is likewise EXPECTED TO PASS. The GENUINELY UNDETERMINED
    /// content of this clause — the part that can teach us something on its first
    /// run — is (b2-ii)'s exact-set equality and (b2-iii)'s 72-coordinate primary
    /// arm. A green G3(iv-b2) that rests only on (b2-i) and (b2-iv) is not
    /// progress.
    ///
    /// BIDIRECTIONALITY, stated explicitly because it is the property the
    /// superseded clause lacked: G3(iv-b2) fails if something the drive CANNOT
    /// resolve is ADMITTED (b2-i, the necessary condition, scored on the admitted
    /// hip capsules), AND it fails if something the drive DOES resolve is
    /// SUPPRESSED (b2-ii, exact-set equality against the witness set). Neither
    /// direction can be satisfied by an empty census, because (b2-iv) pins the
    /// admitted and suppressed counts in both directions: admitting nothing is
    /// RED and admitting everything is RED.
    ///
    /// WHAT THIS SUCCESSOR ASSERTS ABOUT HIPS UNDER A DRIVE THAT IDENTIFIES THEM:
    /// not that they are suppressed, and not merely that they may be admitted —
    /// that each admission is EARNED at every warmed frame by measurable marker
    /// sensitivity to that SPECIFIC coordinate, and that every non-admission
    /// names the coordinate that caused it. WHAT IS NOT PRESERVED, deliberately:
    /// the constant "12 of 12", which was an artefact of a pelvis that never
    /// moved and never was a statement about hips.
    ///
    /// ─── NAVIGATIONAL ADDENDUM, added 2026-08-22 when this successor clause
    /// became EXECUTABLE. Nothing above is edited: no registered sentence, no
    /// bar, no arm, no declared expected verdict, and no `12 of 12`. This note
    /// only says WHERE the text above now runs, and re-derives the ONE figure in
    /// it that the 8.0 s window moved. ───
    /// G3(iv-b2) executes in
    /// `testG3ivB2HipAdmissionIsEarnedPerFrameAndNamesEverySuppressionWitness`,
    /// immediately below this method. G3(iv-a)'s and G3(iv-b)'s own pins,
    /// populations and verdicts are untouched by it: (iv-b) is still executed
    /// here, still measured, and still recorded FAILED PERMANENTLY through
    /// `recordFailedGate` at the bottom of this method.
    /// (b2-iv)'s census quotes **112 warmed frames**; that figure was written at
    /// the 4.0 s window and reads **232** on the regenerated 8.0 s fixtures. The
    /// successor pins 232 and says so at its own execution site. Everything else
    /// in the census — 6/12 suppressed and 14 admitted on `video_012`, 0/12 and
    /// 24 on `video_015` — did NOT move with the window.
    func testG3TheDriveAwareMaskIsNonEmptyOnThePinnedClips() throws {
        // The MEASURED outcome, pinned to the receipt.
        let pinned: [String: (warmed: Int, emptyFrames: Int, hipCapsules: Int,
                              hipSuppressed: Int, admitted: Int)] = [
            "video_012": (232, 0, 12, 6, 14),
            "video_015": (232, 0, 12, 0, 24),
        ]
        for clip in Self.scoredClips {
            let traversal = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g3iv clip=\(clip) warmed=\(traversal.warmedCount) "
                  + "empty_frames=\(traversal.framesWithEmptyUnidentifiedLowerLimb) "
                  + "unidentified_lower_limb=\(traversal.clipUnidentifiedLowerLimb.sorted()) "
                  + "hip_capsules=\(traversal.hipCapsules.count) "
                  + "hip_suppressed=\(traversal.suppressedHipCapsules.count) "
                  + "admitted=\(traversal.admittedCapsules.sorted())")

            let pin = try XCTUnwrap(pinned[clip])
            XCTAssertEqual(traversal.warmedCount, pin.warmed, "G3(iv) pin on \(clip)")
            XCTAssertEqual(traversal.hipCapsules.count, pin.hipCapsules, "G3(iv) pin on \(clip)")
            XCTAssertEqual(traversal.suppressedHipCapsules.count, pin.hipSuppressed,
                           "G3(iv-b) pin on \(clip)")
            XCTAssertEqual(traversal.admittedCapsules.count, pin.admitted, "G3(iv) pin on \(clip)")

            // (iv-a): UNCHANGED and still PASSING, on a non-empty population.
            XCTAssertGreaterThan(traversal.warmedCount, 0,
                                 "G3(iv-a) on \(clip) must not be scored on an empty traversal")
            XCTAssertEqual(traversal.framesWithEmptyUnidentifiedLowerLimb, pin.emptyFrames,
                "G3(iv-a): the drive-aware mask fired on nothing at some warmed frame of \(clip)")
            print("MODE-VERDICT gate=G3(iv-a) clip=\(clip) outcome=PASS_NON_VACUOUS "
                  + "empty_frames=\(traversal.framesWithEmptyUnidentifiedLowerLimb)"
                  + "/\(traversal.warmedCount) bbox_source=macos_vision INTERIM")

            // (iv-b): the clause is NOT met. Recorded, never weakened.
            XCTAssertLessThan(traversal.suppressedHipCapsules.count, traversal.hipCapsules.count,
                "G3(iv-b) is recorded as FAILED on \(clip)")
            recordFailedGate("G3(iv-b)", clip: clip,
                             measured: "hip_suppressed=\(traversal.suppressedHipCapsules.count)"
                                       + "/\(traversal.hipCapsules.count)",
                             bar: "all \(traversal.hipCapsules.count) hip-spanning capsules "
                                  + "suppressed by the fail-closed clip verdict",
                             why: "the clause was registered against the 5-marker drive, where "
                                  + "`hips_joint` never moved; the 20-marker drive identifies the "
                                  + "hip coordinates, so the mask correctly admits them and the "
                                  + "clause correctly fails. A corrected clause is a SUCCESSOR "
                                  + "preregistration, in the class of next-steps 42/43; "
                                  + "bbox_source=macos_vision INTERIM")
        }
    }

    // MARK: - G3(iv-b2): hip admission EARNED per frame, every suppression named

    /// The per-clip EVIDENCE walk `G3(iv-b2)` is scored on.
    ///
    /// Deliberately INDEPENDENT of `buildTraversal`. It re-derives the warmed
    /// smoothed poses from the fixture with the production SG filter and then,
    /// at EVERY warmed frame, takes the marker Jacobian ONCE and reads the SAME
    /// BYTES TWICE:
    ///
    ///  * straight down ONE COLUMN — `‖J·eⱼ‖₂ = √(Σ_r J[r,j]²)` — which is
    ///    (b2-i)'s evidence floor. No Gram matrix, no eigenvector, no
    ///    `nullFraction` is read to produce that number. Asking the mask for its
    ///    own `nullFraction` here would guard NOTHING: the floor is IMPLIED by
    ///    Rule 1, so re-reading Rule 1's output to check Rule 1 is a tautology,
    ///    and the whole value of the clause is that the two computations share
    ///    only the Jacobian bytes;
    ///  * and through `MuscleObservabilityMask.nullFractions`, the mask's own
    ///    Gram/Jacobi route, which is the only thing that can NAME the witness
    ///    (b2-ii) demands per suppressed capsule.
    ///
    /// The two are COMPARED by the gate and never substituted for one another.
    /// The marker list is the CLIP'S OWN `fixture.markerNames`, not
    /// `JointMapping.primary` — that is the difference Rule 1's registered
    /// defect (b) lives in, and it is real here: the clips supply `MHR_ROOT`
    /// where `JointMapping.primary` supplies `PELVIS`.
    struct HipAdmissionEvidence {
        var warmedCount = 0
        var framesMeasured = 0
        var markerCount = 0
        var jacobianRows = 0
        var jacobianColumns = 0
        /// `min_w ‖J(q_w)·eⱼ‖₂` and the warmed frame that attained it.
        var columnNormMinimum: [Double] = []
        var columnNormArgmin: [Int] = []
        /// `max_w nullFractionⱼ(q_w)` and the warmed frame that attained it —
        /// the WORST frame a suppression is attributed to.
        var nullFractionWorst: [Double] = []
        var nullFractionArgmax: [Int] = []
        /// How many warmed frames each coordinate was identified at. PRINTED so
        /// the fail-closed rule is read against the "identified on >= X % of
        /// frames" lever the clause forbids, rather than asserted blind.
        var framesIdentified: [Int] = []
        /// Peak-to-peak over the warmed RAW poses — Rule 2's own quantity, kept
        /// so an UNMEASURED witness is attributable to a number too.
        var coordinateRange: [Double] = []
        /// This walk's own AND-over-frames, beside the shipped rule's answer on
        /// the same per-frame readings. The gate asserts they are ONE set.
        var identifiedByAndOverFrames: Set<Int> = []
        var identifiedByShippedRule: Set<Int> = []
        var unmeasured: Set<Int> = []
        var capsuleSpans: [String: Set<Int>] = [:]
    }

    /// Builds `HipAdmissionEvidence` for one clip.
    static func hipAdmissionEvidence(clip: String, ctx: ModelContext) throws -> HipAdmissionEvidence {
        let bundle = Bundle(for: MuscleLengthModeTests.self)
        let fixture = try SolvedPoseFixture.load(clip: clip, bundle: bundle)
        var out = HipAdmissionEvidence()
        let dofCount = ctx.dofNames.count

        // A PRECONDITION, not a bar: every index below names a coordinate
        // through the LIVE model's order while the poses arrive in the
        // FIXTURE's. `buildTraversal` already depends on those being the same
        // order; if they ever stop being, this gate reads the wrong column and
        // that is a HARNESS defect, never a model finding.
        XCTAssertEqual(fixture.dofNames, ctx.dofNames,
            "HARNESS DEFECT on \(clip), NOT a model finding: the fixture's coordinate order is "
            + "not the live model's, so every column index in G3(iv-b2) would name a different "
            + "coordinate")

        let taps = MuscleLengthModeClassifier.taps
        let halfWindow = taps / 2
        let filters = (0..<dofCount).map { _ in WindowedDerivativeFilter(taps: taps) }
        var smoothedQ: [[Double]] = []
        for (i, pose) in fixture.frames.enumerated() {
            var q = [Double]()
            var warm = true
            for j in 0..<dofCount {
                if let o = filters[j].push(pose[j], timestamp: fixture.timestamps[i]) {
                    q.append(o.pos)
                } else { warm = false }
            }
            guard warm, q.count == dofCount else { continue }
            smoothedQ.append(q)
        }
        out.warmedCount = smoothedQ.count
        guard out.warmedCount > 0 else { return out }

        // Rule 2's own set, over the warmed RAW poses, exactly as the mask takes it.
        let warmedRaw = (0..<out.warmedCount).map { fixture.frames[$0 + halfWindow] }
        out.unmeasured = MuscleObservabilityMask.unmeasuredCoordinates(
            warmedPoses: warmedRaw, coordinateCount: dofCount)
        out.coordinateRange = (0..<dofCount).map { (j: Int) -> Double in
            var lo = Double.infinity
            var hi = -Double.infinity
            for pose in warmedRaw where j < pose.count {
                lo = min(lo, pose[j])
                hi = max(hi, pose[j])
            }
            return hi - lo
        }

        out.markerCount = fixture.markerNames.count
        out.jacobianRows = 3 * fixture.markerNames.count
        out.jacobianColumns = dofCount
        out.columnNormMinimum = [Double](repeating: .infinity, count: dofCount)
        out.columnNormArgmin = [Int](repeating: -1, count: dofCount)
        out.nullFractionWorst = [Double](repeating: -1, count: dofCount)
        out.nullFractionArgmax = [Int](repeating: -1, count: dofCount)
        out.framesIdentified = [Int](repeating: 0, count: dofCount)

        var perFrameNullFractions: [[Double]] = []
        perFrameNullFractions.reserveCapacity(out.warmedCount)
        let rows = out.jacobianRows
        for w in 0..<out.warmedCount {
            guard ctx.setPose(smoothedQ[w]) else {
                XCTFail("G3(iv-b2): the pose did not set on \(clip) warmed frame \(w)")
                continue
            }
            guard let boxed = ctx.bridge.markerPositionJacobian(
                    forMarkerNames: fixture.markerNames) else {
                XCTFail("G3(iv-b2): the marker Jacobian did not resolve on \(clip) warmed "
                        + "frame \(w) — the CLIP's own marker list is what Rule 1 consumes")
                continue
            }
            let flat = boxed.map(\.doubleValue)
            guard flat.count == rows * dofCount else {
                XCTFail("G3(iv-b2): the marker Jacobian on \(clip) frame \(w) is "
                        + "\(flat.count) values, not \(rows * dofCount)")
                continue
            }

            // (b2-i)'s INDEPENDENT CODE PATH. One column, summed down, nothing else.
            for c in 0..<dofCount {
                var sumSquares = 0.0
                for r in 0..<rows {
                    let value = flat[r * dofCount + c]
                    sumSquares += value * value
                }
                let norm = sumSquares.squareRoot()
                if norm < out.columnNormMinimum[c] {
                    out.columnNormMinimum[c] = norm
                    out.columnNormArgmin[c] = w
                }
            }

            // The mask's OWN route, on the SAME bytes.
            guard let fractions = MuscleObservabilityMask.nullFractions(
                    jacobianRowMajor: flat, rows: rows, columns: dofCount) else {
                XCTFail("G3(iv-b2): nullFractions did not resolve on \(clip) frame \(w)")
                continue
            }
            perFrameNullFractions.append(fractions)
            for c in 0..<dofCount where c < fractions.count {
                if fractions[c] > out.nullFractionWorst[c] {
                    out.nullFractionWorst[c] = fractions[c]
                    out.nullFractionArgmax[c] = w
                }
                if MuscleObservabilityMask.isIdentified(nullFraction: fractions[c]) {
                    out.framesIdentified[c] += 1
                }
            }
        }
        out.framesMeasured = perFrameNullFractions.count

        // FRAME-EXHAUSTIVE, computed here and compared against the shipped rule.
        if perFrameNullFractions.isEmpty {
            out.identifiedByAndOverFrames = []
        } else {
            let exhaustive = (0..<dofCount).filter { (j: Int) -> Bool in
                out.framesIdentified[j] == perFrameNullFractions.count
            }
            out.identifiedByAndOverFrames = Set(exhaustive)
        }
        out.identifiedByShippedRule = MuscleObservabilityMask.clipIdentifiedCoordinates(
            perFrameNullFractions: perFrameNullFractions, coordinateCount: dofCount)

        // Rule 0 + the runtime spans, at the SAME first warmed pose the
        // traversal reads them at.
        XCTAssertTrue(ctx.setPose(smoothedQ[0]))
        guard let spanRows = ctx.momentArms(pose: smoothedQ[0], muscles: ctx.displayedMuscles,
                                            coordinates: ctx.dofNames) else {
            throw NSError(domain: "MuscleLengthModeTests", code: 4, userInfo: [
                NSLocalizedDescriptionKey: "G3(iv-b2): runtime spans unavailable on \(clip)"])
        }
        var spanByMuscle: [String: Set<Int>] = [:]
        for (m, name) in ctx.displayedMuscles.enumerated() {
            spanByMuscle[name] = MuscleObservabilityMask.spannedCoordinates(momentArmRow: spanRows[m])
        }
        for resolution in ctx.resolutions where resolution.isResolved {
            var span = Set<Int>()
            for muscle in resolution.modelMuscles { span.formUnion(spanByMuscle[muscle] ?? []) }
            out.capsuleSpans[resolution.capsule] = span
        }
        return out
    }

    /// G3(iv-b2). HIP ADMISSION IS EARNED, PER FRAME, AND EVERY SUPPRESSION
    /// NAMES ITS WITNESS.
    ///
    /// The clause — its four arms, its forced evidence floor, its frame-
    /// exhaustiveness prohibition, its named-witness requirement, its
    /// anti-vacuity arm and its bidirectional census — is registered VERBATIM in
    /// the successor pre-registration above
    /// `testG3TheDriveAwareMaskIsNonEmptyOnThePinnedClips` and is NOT restated
    /// here. This method is that text made EXECUTABLE, nothing more.
    ///
    /// ⚠️ THE DISCLOSURE THE REGISTRATION ATTACHED TO ITSELF IS KEPT AND IS
    /// PRINTED ON THE VERDICT LINE. (b2-i) is a THEOREM under the shipped mask —
    /// `clipIdentifiedCoordinates` is a fail-closed AND over every warmed frame
    /// and `isIdentified` is `nullFraction <= 0.5`, so ADMITTED already forces
    /// `retained >= 0.75` and hence the floor by Bessel. It is EXPECTED TO PASS
    /// and its whole value is as an INDEPENDENT-CODE-PATH guard against an
    /// eigen-route defect or a marker-set mismatch. (b2-iv) re-pins numbers
    /// already asserted at the census pins in
    /// `testG3TheDriveAwareMaskIsNonEmptyOnThePinnedClips` and is likewise
    /// EXPECTED TO PASS. **A GREEN G3(iv-b2) THAT RESTS ONLY ON (b2-i) AND
    /// (b2-iv) IS NOT PROGRESS.** The arms that can teach us something on this
    /// first run are (b2-ii)'s EXACT-SET equality and (b2-iii)'s 72-coordinate
    /// primary arm, and the verdict line carries their populations so a reader
    /// can see whether they were scored on anything.
    ///
    /// G3(iv-b) itself stays FAILED PERMANENTLY and SUPERSEDED-NOT-ERASED: it is
    /// still executed, still measured, and still pinned red through
    /// `recordFailedGate("G3(iv-b)", …)` in the method above. Nothing here
    /// repairs it and nothing here may be read as repairing it.
    ///
    /// ⚠️ ONE FIGURE IN THE REGISTERED TEXT WAS WRITTEN AT THE 4.0 s WINDOW AND
    /// IS RE-DERIVED HERE RATHER THAN COPIED: the census reads **232** warmed
    /// frames per clip, not the 112 the clause quotes. The suppressed and
    /// admitted counts (6/12 and 14; 0/12 and 24) did NOT move with the window.
    /// Receipt for all of them: `MODE-METRIC g3iv` on the 8.0 s fixtures.
    /// PROVENANCE: `bbox_source macos_vision INTERIM` — every census pin here is
    /// provisional ON THE FIXTURES and must be RE-PINNED, not re-interpreted,
    /// when device-grade fixtures land.
    func testG3ivB2HipAdmissionIsEarnedPerFrameAndNamesEverySuppressionWitness() throws {
        let ctx = try context()

        // (b2-i)'s floor, DERIVED from the two shipped constants rather than
        // quoted. `nullFraction <= ceiling` ⇒ `1 − retained <= ceiling²` ⇒
        // `retained >= 1 − ceiling² = 0.75`; Bessel then gives
        // `retained <= ‖J·eⱼ‖² / sigmaVisible²`, hence the floor. NO NEW
        // CONSTANT: change either shipped constant and this turns RED instead of
        // silently sliding the floor with it.
        let ceiling = MuscleObservabilityMask.identifiedNullFractionCeiling
        let sigmaVisible = MuscleObservabilityMask.visibleSingularValueMetresPerRadian
        let retainedFloor = 1.0 - ceiling * ceiling
        let evidenceFloor = retainedFloor.squareRoot() * sigmaVisible
        XCTAssertEqual(ceiling, RegisteredBar.g3IdentifiedNullFractionCeiling,
                       "G3(iv-b2): the identified crossover moved off its registered value")
        XCTAssertEqual(ceiling, PostureFindings.depthSuppressionFraction,
                       "G3(iv-b2): the mask stopped reusing the posture layer's crossover")
        XCTAssertEqual(sigmaVisible, 1.0e-2,
                       "G3(iv-b2): sigmaVisible moved; the floor is its CONSEQUENCE, not a lever")
        XCTAssertEqual(retainedFloor, 0.75,
                       "G3(iv-b2): the forced retained bound is not 0.75")
        XCTAssertEqual(evidenceFloor, 8.660254037844386e-3,
                       "G3(iv-b2): the evidence floor moved off its registered CONSEQUENCE — it "
                       + "may not be tuned")
        XCTAssertLessThan(evidenceFloor,
                          MuscleObservabilityMask.mustNotMaskColumnNormMetresPerRadian,
                          "G3(iv-b2): the floor is not below the measured must-not-mask column "
                          + "norm, so it is over-strict by its own sanity check")
        print(String(format: "MODE-METRIC g3ivb2 evidence_floor=%.15e ceiling=%.6f "
                     + "sigma_visible=%.6e retained_floor=%.6f must_not_mask=%.6f",
                     evidenceFloor, ceiling, sigmaVisible, retainedFloor,
                     MuscleObservabilityMask.mustNotMaskColumnNormMetresPerRadian))

        // (b2-iv) THE BIDIRECTIONAL CENSUS. Re-derived at the 8.0 s window from
        // the `MODE-METRIC g3iv` receipt and the pin table in the method above,
        // NOT copied from the clause text (which was written at 112 warmed).
        // A census that DRIFTS and a census that IMPROVES both turn this RED.
        let census: [String: (warmed: Int, hipCapsules: Int, hipSuppressed: Int,
                              admittedCapsules: Int, admittedHipCapsules: Int,
                              emptyFrames: Int)] = [
            "video_012": (232, 12, 6, 14, 6, 0),
            "video_015": (232, 12, 0, 24, 12, 0),
        ]

        // (b2-iii)'s primary population: the 72 coordinates the 20-marker
        // Jacobian leaves identically zero. G3(ii) asserts this at ONE static
        // `neutral` pose through `JointMapping.primary`; this extends it to the
        // CLIP'S OWN poses and the CLIP'S OWN marker list.
        let unreachableNames = FullBodyDOFFixture.structurallyUnreachableCoordinates
        XCTAssertEqual(unreachableNames.count, 72,
                       "G3(iv-b2)(b2-iii): the structurally-unreachable population is not 72")
        var unreachableIndices: [Int] = []
        for name in unreachableNames {
            guard let j = ctx.dofNames.firstIndex(of: name) else {
                XCTFail("G3(iv-b2)(b2-iii): \(name) is not a live model coordinate")
                continue
            }
            unreachableIndices.append(j)
        }
        XCTAssertEqual(unreachableIndices.count, 72,
                       "G3(iv-b2)(b2-iii): the 72-coordinate arm lost members to name resolution")
        let unreachableSet = Set(unreachableIndices)

        let hipIndices = Set((0..<ctx.dofNames.count).filter { (j: Int) -> Bool in
            ctx.dofNames[j].hasPrefix("hip_")
        })
        XCTAssertGreaterThan(hipIndices.count, 0,
                             "G3(iv-b2): the model has no `hip_` coordinate at all")

        for clip in Self.scoredClips {
            let traversal = try Self.traversal(clip: clip, context: ctx)
            let evidence = try Self.hipAdmissionEvidence(clip: clip, ctx: ctx)
            let pin = try XCTUnwrap(census[clip])
            var failing: [String] = []

            // A walk that warmed NOTHING indexes nothing: every array below is
            // empty, so this is reported and abandoned rather than read.
            guard evidence.warmedCount > 0, evidence.framesMeasured > 0 else {
                var vacuum = "MODE-VERDICT gate=G3(iv-b2) clip=\(clip)"
                vacuum += " outcome=VACUOUS-BY-CONSTRUCTION population=0"
                vacuum += " warmed=\(evidence.warmedCount)"
                vacuum += " frames_measured=\(evidence.framesMeasured)"
                vacuum += " reason=the_independent_walk_warmed_no_frame"
                vacuum += " note=0_==_0_scores_nothing__NOT_a_pass"
                print(vacuum)
                XCTFail("G3(iv-b2) on \(clip): the independent evidence walk warmed no frame, "
                        + "so no arm of this clause was scored on anything")
                continue
            }

            // The union of the TWELVE hip capsules' spans. `unmeasured` is a
            // model-wide set, so printing it whole is noise; this is the only
            // part of the trigger set that can suppress a hip capsule at all.
            var hipSpanUnion = Set<Int>()
            for capsule in traversal.hipCapsules {
                hipSpanUnion.formUnion(evidence.capsuleSpans[capsule] ?? [])
            }

            // ── COHERENCE. This walk and the shared traversal must be looking
            // at the same frames; if they are not, nothing below means anything
            // and the cause is a HARNESS defect, not a model finding.
            let warmedAgrees = evidence.warmedCount == traversal.warmedCount
            let measuredAll = evidence.framesMeasured == evidence.warmedCount
            if !warmedAgrees || !measuredAll { failing.append("harness_walk_disagrees") }

            // ── (b2-ii) FRAME-EXHAUSTIVENESS. The shipped clip verdict must BE
            // the AND over every warmed frame. This is where an "identified on
            // >= X % of frames" lever would first become visible.
            let exhaustiveAgrees =
                evidence.identifiedByShippedRule == evidence.identifiedByAndOverFrames
            if !exhaustiveAgrees { failing.append("b2ii_frame_exhaustiveness") }

            let lowerLimbNames = Set(ctx.table.coordinateNames.prefix(20))
            let unidentifiedLowerLimbNames = traversal.clipUnidentifiedLowerLimb
            var unidentifiedLowerLimb: Set<Int> = []
            for name in unidentifiedLowerLimbNames {
                if let j = ctx.dofNames.firstIndex(of: name) { unidentifiedLowerLimb.insert(j) }
            }
            let lowerLimbIndices = Set(lowerLimbNames.compactMap { (name: String) -> Int? in
                ctx.dofNames.firstIndex(of: name)
            })
            let derivedUnidentifiedLowerLimb =
                lowerLimbIndices.subtracting(evidence.identifiedByShippedRule)
            let lowerLimbAgrees = derivedUnidentifiedLowerLimb == unidentifiedLowerLimb
            if !lowerLimbAgrees { failing.append("harness_lower_limb_disagrees") }

            // The witness set the clause names: `clipUnidentifiedLowerLimb ∪ unmeasured`.
            let witnessTrigger = unidentifiedLowerLimb.union(evidence.unmeasured)

            // ── (b2-i) THE EVIDENCE FLOOR, on the ADMITTED hip capsules.
            let admittedHipCapsules = traversal.hipCapsules
                .filter { (capsule: String) -> Bool in
                    traversal.admittedCapsules.contains(capsule)
                }
                .sorted()
            var floorCells = 0
            var floorWorstNorm = Double.infinity
            var floorWorstCoordinate = "none"
            var floorWorstFrame = -1
            var floorWorstCapsule = "none"
            var floorBreaches: [String] = []
            for capsule in admittedHipCapsules {
                let spanned = evidence.capsuleSpans[capsule] ?? []
                let spannedHips = spanned.intersection(hipIndices).sorted()
                for j in spannedHips {
                    floorCells += evidence.framesMeasured
                    let norm = evidence.columnNormMinimum[j]
                    if norm < floorWorstNorm {
                        floorWorstNorm = norm
                        floorWorstCoordinate = ctx.dofNames[j]
                        floorWorstFrame = evidence.columnNormArgmin[j]
                        floorWorstCapsule = capsule
                    }
                    if norm < evidenceFloor {
                        floorBreaches.append("\(capsule)/\(ctx.dofNames[j])"
                                             + "@w\(evidence.columnNormArgmin[j])")
                    }
                }
                var line = "MODE-METRIC g3ivb2-admitted clip=\(clip) capsule=\(capsule)"
                line += " spanned_hip_coordinates=\(spannedHips.map { ctx.dofNames[$0] })"
                line += " spanned_total=\(spanned.count)"
                let norms = spannedHips.map { evidence.columnNormMinimum[$0] }
                line += String(format: " min_column_norm=%.9e", norms.min() ?? -1.0)
                let worstFractions = spannedHips.map { evidence.nullFractionWorst[$0] }
                line += String(format: " worst_null_fraction=%.9f", worstFractions.max() ?? -1.0)
                print(line)
            }
            let floorVacuous = floorCells == 0
            if floorVacuous {
                var vacuum = "MODE-VERDICT gate=G3(iv-b2)(b2-i)"
                vacuum += " outcome=VACUOUS-BY-CONSTRUCTION population=0 clip=\(clip)"
                vacuum += " admitted_hip_capsules=\(admittedHipCapsules.count)"
                vacuum += " reason=the_drive_admitted_no_hip_capsule_so_the_floor_scored_nothing"
                vacuum += " note=0_==_0_scores_nothing__NOT_a_pass"
                vacuum += "__NOT_evidence_admission_was_earned"
                print(vacuum)
                failing.append("b2i_vacuous")
            } else if !floorBreaches.isEmpty {
                failing.append("b2i_evidence_floor")
            }

            // ── (b2-ii) EXACT-SET EQUALITY plus a NAMED WITNESS per suppression.
            let suppressedHips = Set(traversal.suppressedHipCapsules)
            let expectedSuppressedHips = Set(traversal.hipCapsules
                .filter { (capsule: String) -> Bool in
                    !(evidence.capsuleSpans[capsule] ?? []).isDisjoint(with: witnessTrigger)
                })
            let exactSetAgrees = suppressedHips == expectedSuppressedHips
            if !exactSetAgrees { failing.append("b2ii_exact_set") }
            var unnamedSuppressions: [String] = []
            for capsule in traversal.suppressedHipCapsules {
                let spanned = evidence.capsuleSpans[capsule] ?? []
                let triggers = spanned.intersection(witnessTrigger)
                let ranked = triggers.sorted { (a: Int, b: Int) -> Bool in
                    if evidence.nullFractionWorst[a] != evidence.nullFractionWorst[b] {
                        return evidence.nullFractionWorst[a] > evidence.nullFractionWorst[b]
                    }
                    return ctx.dofNames[a] < ctx.dofNames[b]
                }
                guard let witness = ranked.first else {
                    unnamedSuppressions.append(capsule)
                    var orphan = "MODE-METRIC g3ivb2-witness clip=\(clip) capsule=\(capsule)"
                    orphan += " witness=NONE"
                    orphan += " reason=suppressed_by_a_coordinate_OUTSIDE"
                    orphan += "_clipUnidentifiedLowerLimb_union_unmeasured"
                    orphan += " adjudicate=[the hip block is being greyed from outside the lower"
                    orphan += " limb — a FINDING, never repaired by widening the witness set]"
                    print(orphan)
                    continue
                }
                let worstFrame = evidence.nullFractionArgmax[witness]
                var line = "MODE-METRIC g3ivb2-witness clip=\(clip) capsule=\(capsule)"
                line += " witness=\(ctx.dofNames[witness])"
                var rule = "rule1_unidentified"
                if evidence.identifiedByShippedRule.contains(witness) { rule = "rule2_unmeasured" }
                line += " rule=\(rule)"
                line += " worst_warmed_frame=\(worstFrame)"
                line += String(format: " null_fraction=%.9f", evidence.nullFractionWorst[witness])
                line += String(format: " column_norm=%.9e", evidence.columnNormMinimum[witness])
                line += String(format: " coordinate_range=%.9e", evidence.coordinateRange[witness])
                line += " frames_identified=\(evidence.framesIdentified[witness])"
                line += "/\(evidence.framesMeasured)"
                line += " other_triggers=\(ranked.dropFirst().map { ctx.dofNames[$0] })"
                print(line)
            }
            if !unnamedSuppressions.isEmpty { failing.append("b2ii_unnamed_witness") }

            // The fail-closed rule, READ rather than assumed: how many warmed
            // frames each lower-limb coordinate was identified at. A coordinate
            // sitting at, say, 231/232 is exactly what an "identified on >= X %
            // of frames" lever would rescue, and the clause forbids one.
            for name in ctx.table.coordinateNames.prefix(20) {
                guard let j = ctx.dofNames.firstIndex(of: name) else { continue }
                var line = "MODE-METRIC g3ivb2-lowerlimb clip=\(clip) coordinate=\(name)"
                line += " frames_identified=\(evidence.framesIdentified[j])"
                line += "/\(evidence.framesMeasured)"
                line += String(format: " worst_null_fraction=%.9f", evidence.nullFractionWorst[j])
                line += String(format: " min_column_norm=%.9e", evidence.columnNormMinimum[j])
                line += String(format: " range=%.9e", evidence.coordinateRange[j])
                line += " unmeasured=\(evidence.unmeasured.contains(j))"
                line += " clip_identified=\(evidence.identifiedByShippedRule.contains(j))"
                print(line)
            }

            // ── (b2-iii) ANTI-VACUITY. PRIMARY arm: all 72 unidentified at EVERY
            // warmed frame, on the clip's own poses and marker list.
            var identifiedUnreachable: [String] = []
            for j in unreachableIndices where evidence.framesIdentified[j] > 0 {
                identifiedUnreachable.append("\(ctx.dofNames[j])"
                                             + ":\(evidence.framesIdentified[j])"
                                             + "/\(evidence.framesMeasured)")
            }
            let unreachableCells = unreachableIndices.count * evidence.framesMeasured
            if !identifiedUnreachable.isEmpty { failing.append("b2iii_primary") }

            // COMPANION arm, population declared rather than discovered.
            let capsulesReachingUnreachable = evidence.capsuleSpans.keys
                .filter { (capsule: String) -> Bool in
                    !(evidence.capsuleSpans[capsule] ?? []).isDisjoint(with: unreachableSet)
                }
                .sorted()
            let companionUnsuppressed = capsulesReachingUnreachable
                .filter { (capsule: String) -> Bool in
                    traversal.admittedCapsules.contains(capsule)
                }
            if capsulesReachingUnreachable.isEmpty {
                var vacuum = "MODE-VERDICT gate=G3(iv-b2)(b2-iii)-companion"
                vacuum += " outcome=VACUOUS-BY-CONSTRUCTION population=0 clip=\(clip)"
                vacuum += " resolved_capsules=\(evidence.capsuleSpans.count)"
                vacuum += " reason=no_declared_capsule_SPANS_one_of_the_72"
                vacuum += "__ercspn_r_and_ercspn_l_resolve_to_ZERO_model_muscles_and_psoas_runs"
                vacuum += "_pelvis_to_femur_only"
                vacuum += " note=0_==_0_scores_nothing__NOT_a_pass"
                vacuum += "__NOT_evidence_trunk_capsules_are_suppressed"
                print(vacuum)
            } else if !companionUnsuppressed.isEmpty {
                failing.append("b2iii_companion")
            }

            // ── (iv-a) RETAINED UNCHANGED: "fires on 0" is still DISPROOF.
            if traversal.framesWithEmptyUnidentifiedLowerLimb != pin.emptyFrames {
                failing.append("iv_a_mask_fired_on_nothing")
            }

            // ── (b2-iv) THE CENSUS, both directions.
            var censusDrift: [String] = []
            if traversal.warmedCount != pin.warmed { censusDrift.append("warmed") }
            if traversal.hipCapsules.count != pin.hipCapsules { censusDrift.append("hip_capsules") }
            if traversal.suppressedHipCapsules.count != pin.hipSuppressed {
                censusDrift.append("hip_suppressed")
            }
            if traversal.admittedCapsules.count != pin.admittedCapsules {
                censusDrift.append("admitted")
            }
            if admittedHipCapsules.count != pin.admittedHipCapsules {
                censusDrift.append("admitted_hip_capsules")
            }
            if !censusDrift.isEmpty { failing.append("b2iv_census") }

            // ── THE RECEIPT, printed BEFORE the assertions so a red run is still
            // a complete reading.
            var metric = "MODE-METRIC g3ivb2 clip=\(clip) warmed=\(evidence.warmedCount)"
            metric += " frames_measured=\(evidence.framesMeasured)"
            metric += " traversal_warmed=\(traversal.warmedCount)"
            metric += " markers=\(evidence.markerCount) jacobian=\(evidence.jacobianRows)"
            metric += "x\(evidence.jacobianColumns)"
            metric += " hip_coordinates=\(hipIndices.count)"
            metric += " hip_capsules=\(traversal.hipCapsules.count)"
            metric += " admitted_hip_capsules=\(admittedHipCapsules.count)"
            metric += " suppressed_hip_capsules=\(traversal.suppressedHipCapsules.count)"
            metric += " b2i_cells=\(floorCells)"
            metric += String(format: " b2i_min_column_norm=%.9e", floorWorstNorm)
            metric += " b2i_min_at=\(floorWorstCapsule)/\(floorWorstCoordinate)"
            metric += "@w\(floorWorstFrame)"
            metric += String(format: " b2i_margin_x=%.4f", floorWorstNorm / evidenceFloor)
            metric += " b2i_breaches=\(floorBreaches)"
            metric += " b2ii_suppressed=\(traversal.suppressedHipCapsules)"
            metric += " b2ii_expected=\(expectedSuppressedHips.sorted())"
            metric += " b2ii_witness_trigger_total=\(witnessTrigger.count)"
            metric += " b2ii_witness_trigger_on_hip_spans="
            metric += "\(witnessTrigger.intersection(hipSpanUnion).sorted().map { ctx.dofNames[$0] })"
            metric += " b2ii_unidentified_lower_limb="
            metric += "\(unidentifiedLowerLimb.sorted().map { ctx.dofNames[$0] })"
            metric += " b2ii_unnamed=\(unnamedSuppressions)"
            metric += " b2iii_primary_cells=\(unreachableCells)"
            metric += " b2iii_identified_unreachable=\(identifiedUnreachable)"
            metric += " b2iii_companion_population=\(capsulesReachingUnreachable.count)"
            metric += " census_drift=\(censusDrift)"
            print(metric)

            let outcome = failing.isEmpty ? "PASS_NON_VACUOUS" : "FAILED_AGAINST_REGISTERED_BAR"
            var verdict = "MODE-VERDICT gate=G3(iv-b2) clip=\(clip) outcome=\(outcome)"
            verdict += " failing=\(failing)"
            verdict += " arms=[(b2-i) evidence floor;(b2-ii) exact set + named witness;"
            verdict += "(b2-iii) 72-coordinate anti-vacuity;(b2-iv) bidirectional census]"
            verdict += " registered_bar=[every admitted hip capsule's every spanned hip"
            verdict += " coordinate clears 8.660254037844386e-3 m/rad at EVERY warmed frame;"
            verdict += " suppressed-hip set == hip capsules spanning"
            verdict += " clipUnidentifiedLowerLimb union unmeasured, each with a NAMED witness;"
            verdict += " all 72 structurally-unreachable coordinates UNIDENTIFIED at every warmed"
            verdict += " frame; census pinned in BOTH directions]"
            verdict += " known_pass_declared_before_run=[(b2-i) is a THEOREM under the shipped"
            verdict += " mask — its value is the INDEPENDENT CODE PATH, not the reading;"
            verdict += " (b2-iv) re-pins numbers already asserted at the G3(iv) census]"
            verdict += " undetermined=[(b2-ii) exact-set equality;(b2-iii) primary arm]"
            verdict += " undetermined_populations=[b2ii_hip_capsules=\(traversal.hipCapsules.count)"
            verdict += ";b2ii_suppressed=\(traversal.suppressedHipCapsules.count)"
            verdict += ";b2iii_cells=\(unreachableCells)]"
            // PER-ARM VACUITY, added 2026-08-22 by adversarial review before this
            // gate had ever run. On a clip that suppresses NO hip capsule, the
            // named-witness arm and the exact-set arm are both empty-set
            // comparisons, and the top-level token would read PASS_NON_VACUOUS
            // over them. The arm's own vacuity has to be legible in the receipt,
            // because (b2-ii) is one of only two arms registered as able to
            // teach anything on a first run.
            verdict += traversal.suppressedHipCapsules.isEmpty
                ? " b2ii_named_witness_arm=VACUOUS-BY-CONSTRUCTION__0_suppressions_on_this_clip"
                  + "__the_exact_set_comparison_above_is_empty_==_empty__NOT_a_pass"
                : " b2ii_named_witness_arm=SCORED"
                  + " population=\(traversal.suppressedHipCapsules.count)"
            verdict += " NOT_PROGRESS_IF=[a green that rests only on (b2-i) and (b2-iv)"
            verdict += " — those two were determinable before the run and neither is evidence"
            verdict += " hip admission was earned]"
            verdict += " independent_code_path=[column norm summed down ONE column of the same"
            verdict += " markerPositionJacobian bytes; the mask's own nullFraction is NEVER read"
            verdict += " to produce it]"
            verdict += " supersedes=[G3(iv-b) — FAILED PERMANENTLY, SUPERSEDED-NOT-ERASED, still"
            verdict += " executed and still pinned red in"
            verdict += " testG3TheDriveAwareMaskIsNonEmptyOnThePinnedClips]"
            verdict += " claims_not=[that hips are suppressed, and that 12-of-12 means anything"
            verdict += " — it was an artefact of a pelvis that never moved]"
            verdict += " first_measurement=true window_seconds=8.0"
            verdict += " bbox_source=macos_vision INTERIM"
            print(verdict)

            if !failing.isEmpty {
                recordFailedGate("G3(iv-b2)", clip: clip,
                                 measured: "failing=\(failing); b2i_cells=\(floorCells); "
                                           + "suppressed=\(traversal.suppressedHipCapsules); "
                                           + "expected=\(expectedSuppressedHips.sorted()); "
                                           + "census_drift=\(censusDrift)",
                                 bar: "admission earned at every warmed frame, every suppression "
                                      + "named, all 72 unreachable coordinates unidentified, "
                                      + "census pinned both ways",
                                 why: "ADJUDICATE, never absorb: a (b2-ii) red means the hip "
                                      + "block is being greyed from OUTSIDE the lower limb and "
                                      + "may NOT be repaired by widening the witness set; a "
                                      + "(b2-i) red means the eigen route or the marker set, not "
                                      + "the floor, which is a CONSEQUENCE of two shipped "
                                      + "constants; bbox_source=macos_vision INTERIM")
            }

            // ── THE BARS, asserted last so every receipt above is already out.
            XCTAssertEqual(evidence.warmedCount, traversal.warmedCount,
                "G3(iv-b2) HARNESS: this walk and the shared traversal warmed differently on "
                + "\(clip) — nothing measured here is comparable until that is repaired")
            XCTAssertEqual(evidence.framesMeasured, evidence.warmedCount,
                "G3(iv-b2) HARNESS: a warmed frame on \(clip) produced no Jacobian reading")
            XCTAssertEqual(evidence.identifiedByShippedRule, evidence.identifiedByAndOverFrames,
                "G3(iv-b2)(b2-ii): the shipped clip verdict is no longer the AND over EVERY "
                + "warmed frame on \(clip) — an `identified on >= X % of frames` lever is "
                + "exactly what this bar exists to catch, and no successor may introduce one")
            XCTAssertEqual(derivedUnidentifiedLowerLimb, unidentifiedLowerLimb,
                "G3(iv-b2) HARNESS: the independent walk's unidentified lower-limb block differs "
                + "from the shared traversal's on \(clip)")

            // (b2-i), on a population this gate refuses to score if it is empty.
            XCTAssertGreaterThan(floorCells, 0,
                "G3(iv-b2)(b2-i): VACUOUS — the drive admitted no hip capsule on \(clip), so the "
                + "evidence floor scored NOTHING. 0 == 0 is not a pass, and admitting nothing is "
                + "RED by the clause's own bidirectionality")
            XCTAssertEqual(floorBreaches, [],
                "G3(iv-b2)(b2-i): an ADMITTED hip capsule on \(clip) spans a hip coordinate whose "
                + "marker-Jacobian column norm falls below the forced floor "
                + "\(evidenceFloor) m/rad. Rule 1 says that cannot happen, so this is an "
                + "eigen-route defect or a marker-set mismatch, NOT a reason to move the floor")

            // (b2-ii).
            XCTAssertEqual(suppressedHips, expectedSuppressedHips,
                "G3(iv-b2)(b2-ii): the suppressed-hip set on \(clip) is not EXACTLY the hip "
                + "capsules spanning clipUnidentifiedLowerLimb union unmeasured. A capsule "
                + "suppressed by a coordinate OUTSIDE the lower-limb block is a FINDING to be "
                + "adjudicated, never absorbed and never repaired by widening the witness set")
            XCTAssertEqual(unnamedSuppressions, [],
                "G3(iv-b2)(b2-ii): a suppressed hip capsule on \(clip) has no nameable witness. "
                + "A mask that greys half a block for no nameable reason is not a derivation")

            // (b2-iii) primary, on 72 x warmed cells.
            XCTAssertGreaterThan(unreachableCells, 0,
                "G3(iv-b2)(b2-iii): the primary arm scored no cells on \(clip)")
            XCTAssertEqual(identifiedUnreachable, [],
                "G3(iv-b2)(b2-iii): a structurally-unreachable coordinate reads IDENTIFIED at a "
                + "warmed frame of \(clip) on the clip's own marker list — G3(ii) only ever "
                + "checked one static pose through JointMapping.primary")
            XCTAssertEqual(companionUnsuppressed, [],
                "G3(iv-b2)(b2-iii) companion: a capsule spanning a structurally-unreachable "
                + "coordinate was ADMITTED on \(clip)")

            // (iv-a), unchanged.
            XCTAssertEqual(traversal.framesWithEmptyUnidentifiedLowerLimb, pin.emptyFrames,
                "G3(iv-a) on \(clip): the drive-aware mask fired on nothing at some warmed frame")

            // (b2-iv), both directions.
            XCTAssertEqual(censusDrift, [],
                "G3(iv-b2)(b2-iv): the admission census on \(clip) moved. A census that DRIFTS "
                + "and a census that IMPROVES both force a fresh adjudication rather than a "
                + "silent re-baseline; these pins are provisional ON macos_vision INTERIM "
                + "fixtures and must be RE-PINNED when device-grade fixtures land")
            XCTAssertEqual(traversal.warmedCount, pin.warmed, "G3(iv-b2)(b2-iv) warmed on \(clip)")
            XCTAssertEqual(traversal.hipCapsules.count, pin.hipCapsules,
                           "G3(iv-b2)(b2-iv) hip capsules on \(clip)")
            XCTAssertEqual(traversal.suppressedHipCapsules.count, pin.hipSuppressed,
                           "G3(iv-b2)(b2-iv) suppressed on \(clip)")
            XCTAssertEqual(traversal.admittedCapsules.count, pin.admittedCapsules,
                           "G3(iv-b2)(b2-iv) admitted on \(clip)")
            XCTAssertEqual(admittedHipCapsules.count, pin.admittedHipCapsules,
                           "G3(iv-b2)(b2-iv) admitted hip capsules on \(clip)")
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
    /// VERDICT (2026-08-13, 5-marker lineage): G3(v) FAILED against its
    /// registered bar (`neutral` read 0.832149 > 0.5 on both knees). Reopening
    /// needed richer fixtures — a production-grade 20-marker solved-pose clip —
    /// plus a fresh adjudication.
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ─── **The richer fixture arrived and the verdict FLIPPED to
    /// PASS.** The bar is untouched (`identified iff null_fraction <= 0.5`, read
    /// from the SHIPPED `MuscleObservabilityMask.identifiedNullFractionCeiling`);
    /// what moved is the DRIVE. Reading the marker set out of the 20-marker
    /// fixture, `neutral` falls from **0.832149 to 0.000688** on both knees —
    /// three orders of magnitude — and every one of the six measured null
    /// fractions is now ≤ 0.000812. The failing pose is identified; `failures`
    /// is empty. Receipt: `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g3v'`.
    ///
    /// ⚠️ **THE METHOD NAME IS NOW STALE and is deliberately not renamed.** It
    /// says "UnderTheFiveMarkerDrive"; since this round the drive it reads is
    /// the TWENTY-marker one. The MODE instrument is frozen for this round, so
    /// the correction is recorded here rather than applied silently; renaming it
    /// is a successor-round item.
    ///
    /// PROVENANCE: macOS Vision, INTERIM. The knee being identified is a
    /// statement about the MARKER SET, not about the phone.
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

        // The MEASURED outcome, pinned to the receipt. TRANSITIONED 2026-08-14
        // from the 5-marker lineage's 0.832149 / 0.297860 / 0.113131 set.
        let pinned: [String: Double] = [
            "neutral/knee_angle_r": 0.000688,
            "neutral/knee_angle_l": 0.000688,
            "run_1_midstance/knee_angle_r": 0.000460,
            "run_1_midstance/knee_angle_l": 0.000812,
            "run_4_mid_swing/knee_angle_r": 0.000812,
            "run_4_mid_swing/knee_angle_l": 0.000461,
        ]
        XCTAssertEqual(Set(measured.keys), Set(pinned.keys),
                       "the G3(v) population itself moved")
        for (key, expected) in pinned {
            let value = try XCTUnwrap(measured[key])
            XCTAssertEqual(value, expected, accuracy: 1.0e-6,
                           "G3(v) pin \(key): the measured null fraction moved off its receipt")
        }

        // NON-VACUITY: the bar is read against SIX measured fractions from a
        // resolved marker Jacobian, not against an empty set.
        XCTAssertEqual(markers.count, 20,
                       "G3(v) now reads the TWENTY-marker drive; the method's name is stale and "
                       + "the correction is recorded in its doc comment rather than applied "
                       + "silently while the MODE instrument is frozen")
        XCTAssertEqual(measured.count, 6, "G3(v) must score six coordinate/pose cells")

        // The VERDICT: the gate PASSES. Every pose is identified, including the
        // straight-leg `neutral` that failed under the 5-marker drive.
        XCTAssertEqual(failures, [],
            "G3(v) PASSES: with twenty markers every scored pose is identified, including the "
            + "straight-leg `neutral` that read 0.832149 under the 5-marker drive")
        for pose in ["neutral", "run_1_midstance", "run_4_mid_swing"] {
            for name in ["knee_angle_r", "knee_angle_l"] {
                let value = try XCTUnwrap(measured["\(pose)/\(name)"])
                XCTAssertTrue(MuscleObservabilityMask.isIdentified(nullFraction: value),
                              "\(pose)/\(name) must be identified")
                XCTAssertLessThan(value, RegisteredBar.g3IdentifiedNullFractionCeiling,
                                  "\(pose)/\(name) must sit BELOW the crossover")
            }
        }
        let neutral = try XCTUnwrap(measured["neutral/knee_angle_r"])
        print(String(format: "MODE-VERDICT gate=G3(v) outcome=PASS_NON_VACUOUS "
                     + "neutral=%.6f worst=%.6f bar=<= %.1f markers=%d "
                     + "bbox_source=macos_vision INTERIM "
                     + "note=the BAR did not move; the DRIVE did. 0.832149 -> %.6f at `neutral`.",
                     neutral, measured.values.max() ?? 0,
                     RegisteredBar.g3IdentifiedNullFractionCeiling, markers.count, neutral))
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
    ///
    /// ─── SUCCESSOR PRE-REGISTRATION, written 2026-08-21, RED-FIRST, BEFORE the
    /// measurement that adjudicates it ───
    ///
    /// **G4(a) stands FAILED and is SUPERSEDED-NOT-ERASED BY G4(f) + G4(g).** The
    /// text above is not edited, the verdict is not re-adjudicated, the pins are
    /// not re-baselined, and `RegisteredBar.g4DirectionMatch` keeps the value
    /// 0.95. The registered sweep endpoints (knee 0→140, ankle −30→20, hip
    /// −20→120, elbow 0→150 deg — `g4Sweeps`, :954-972), the 26-anchor set and
    /// the frozen exclusion list are NOT touched by this successor. Shortening a
    /// sweep after seeing a failure is the laundering move the registration froze
    /// out; it is refused here in writing, and nothing below may be read as
    /// making it.
    ///
    /// WHY A SUCCESSOR EXISTS. G4(a) asserts ONE proposition on TWO independent
    /// premises: (1) the PORT reproduces the SHIPPED MODEL's direction, and
    /// (2) the SHIPPED MODEL reproduces the TEXTBOOK direction. The measurement
    /// falsified (2) and left (1) UNTESTED — `g4Sweep` computes moment arms
    /// through `ctx.momentArms` (:1343-1354) and never opens the oracle at all. A
    /// gate whose two premises cannot fail separately cannot attribute its own
    /// failure. G4(f) and G4(g) split them so they fail separately and mean
    /// different things. NOTHING G4(a) GUARDED IS DROPPED: its own pins keep the
    /// constructed-pose measurement, G4(f) adds the port-vs-model proposition it
    /// never had, and G4(g) carries the textbook proposition for all 26 anchors —
    /// as an EMPTY register for 22 of them and as a pinned, enumerated model
    /// defect for the other 4.
    ///
    /// NOT REGISTERED, AND WHY, recorded so it is not silently re-proposed:
    /// restricting G4's population to the anchors on the Rule-0 product surface.
    /// All four failing anchors are ALREADY off that surface —
    /// `MuscleOverlay.muscleDefs` declares 26 capsules (12 `bilateral(...)` calls
    /// plus `ercspn_r`/`ercspn_l`, MuscleOverlay.swift:134-247) and not one of
    /// them is a triceps or a `bfsh`, and Rule 0's family expansion matches on an
    /// EXACT stem (MuscleObservabilityMask.swift:248-266), so `bflh` can never
    /// reach `bfsh` — therefore that clause reads 1.000000 BEFORE it runs.
    /// Narrowing a failing gate's population to the subset where it already
    /// passes is laundering however it is disclosed, and it is refused. Also
    /// refused: swapping the anchor for a muscle the model does honour, which
    /// selects anchors FOR agreement and would erase the one product-relevant
    /// thing this round measured — that the shipped model is anatomically wrong
    /// somewhere.
    ///
    /// **G4(f) PORT FIDELITY, FULL 26-ANCHOR SET.** Population: ALL 26 registered
    /// anchors, displayed or not, scored on the ORACLE'S OWN sweep poses in
    /// `BioMotionTests/Fixtures/opensim_moment_arms.txt` — the oracle holds only
    /// its own poses, so this gate cannot be run on G4's constructed ones. It
    /// asserts that the PORT's computed direction agrees with the MODEL's own
    /// direction, cell by cell. It carries NO textbook claim and cannot be
    /// repaired by editing an anchor: it fails if and only if the port stops
    /// reproducing the shipped model.
    /// Registered construction, re-derived by hand from the fixture's own pose
    /// rows on 2026-08-21 and pinned here before the run:
    ///   * grids — knee 29 poses 0..140 deg step 5.0 (28 adjacent pairs); hip 29
    ///     poses −20..120 step 5.0 (28); ankle 29 poses −40..+30 step 2.5 (28);
    ///     elbow 16 poses 0..150 step 10.0 (15). 10·28 + 4·28 + 5·28 + 7·15 = 637
    ///     scored anchor-pairs. The shoulder sweep carries no registered anchor
    ///     and stays excluded, exactly as frozen at :950-953.
    ///   * the oracle's BASE POSES differ from G4's `neutral` — its elbow sweep
    ///     carries `shoulder_elv_r = 30 deg`, its knee sweep `hip_flexion_r =
    ///     20 deg` — and that is a FEATURE, not a difference to normalise away:
    ///     an agreement that survives a base-pose change is stronger than one
    ///     that does not.
    ///   * MODEL reference column, WRAP-ON, NAMED: `lengthWrapOn(b) −
    ///     lengthWrapOn(a)` for muscles carrying < 2 PathWraps, and the
    ///     pair-averaged analytic column `−0.5·(R_on(a) + R_on(b))·dq` for
    ///     muscles carrying ≥ 2 — the same multi-wrap split G1 already registers,
    ///     for the same stated reason. The WRAP-OFF columns (`roff` / `Loff`) are
    ///     NOT the reference and may not be substituted: on those columns the
    ///     conflict picture is a completely different 7-anchor set (see G4(g)),
    ///     so an implementer reading the wrong field would produce a
    ///     self-consistent but wrong gate.
    /// Bars, registered before measurement: ≥ 99.0 % port-vs-model direction
    /// agreement per anchor; ZERO disagreements on pairs where the reference
    /// |dL| ≥ 10× that pair's step deadband; coverage ≥ 60 % of an anchor's
    /// spanning pairs. NON-VACUITY, asserted not assumed: every anchor must score
    /// > 0 pairs, and an anchor scoring none is RED, not skipped.
    /// DISCLOSURE, added 2026-08-21 by adversarial review BEFORE any measurement:
    /// at these population sizes the 99.0 % bar IS a ZERO-TOLERANCE bar and must
    /// be read as one. The largest anchor population is 28 pairs (knee / hip /
    /// ankle) and the elbow's is 15; 27/28 = 96.43 % and 14/15 = 93.33 % both
    /// MISS 99 %, so a single disagreeing pair fails the anchor. No minimum-pairs
    /// power floor is registered here — unlike G1, which carries one
    /// (`strongCells >= 250`) — and NONE MAY BE ADDED AFTER A MISS: adding a
    /// power floor in response to a near miss is the same move as widening a bar.
    /// The companion 10×-deadband arm is near-degenerate for the same reason: on
    /// the fixture face `sigma = 1e-6` rad puts the deadband near 5e-8 m against
    /// measured |dL| of 1.9e-4 … 3.5e-3 m, so essentially the whole population is
    /// a strong cell and that arm restates the first rather than adding power.
    /// The bars are LEFT UNCHANGED; this paragraph changes only what a reader is
    /// told they mean.
    /// WHAT IT LETS THE PRODUCT CLAIM: that what the layer would draw is what the
    /// shipped model says, on a population 13 anchors wider than G1's —
    /// `TRIlong_r`, `BIClong_r`, `grac_r`, `sart_r`, `iliacus_r`, `vasint_r`,
    /// `semiten_r` and `bfsh140_r` have never been compared to the oracle ON THE
    /// REGISTERED ANCHOR SWEEPS BY G1, whose `g1Outcome` filters to
    /// `ctx.displayedMuscles`.
    /// CORRECTION, 2026-08-21, by adversarial review, SUPERSEDING the first
    /// draft of this sentence which read "have never been compared to the oracle
    /// by anything": that was FALSE for 7 of those 8. All except `sart_r` carry a
    /// PathWrap (`opensim_moment_arms.txt` :208 TRIlong_r, :190 BIClong_r,
    /// :258 grac_r, :260 iliacus_r, :286 vasint_r, :276 semiten_r, :226
    /// bfsh140_r = 1; only :272 sart_r = 0), so they sit inside the 66
    /// wrap-carrying muscles that `CylinderWrapValidationTests` (56 cylinder-only)
    /// and `EllipsoidWrapValidationTests` (10 ellipsoid, multi-wrap stratum =
    /// exactly BIClong / TRIlong) ALREADY gate against OpenSim's own central
    /// difference. Only `sart_r` (carriesPathWrap = 0, absent from
    /// `opensim_moment_arms_fd.txt`) is genuinely uncompared. G4(f)'s novelty is
    /// therefore the ANCHOR-SWEEP FACE and the direction-agreement framing, NOT
    /// first contact with the oracle, and it must not be quoted as the latter.
    /// WHAT IT DOES NOT CLAIM: one word about anatomy. That is G4(g).
    ///
    /// **G4(g) MODEL-ANCHOR CONFLICT REGISTER.** Derived from the ORACLE ALONE —
    /// the port is not an input, so no change to the port can move this gate.
    /// Definition, with its convention PINNED because the answer depends on it: a
    /// CONFLICT CELL is an adjacent oracle sweep pair at whose PAIR MIDPOINT
    /// `|dL| > MuscleLengthModeClassifier.lengthQuantisationFloorMetres`
    /// (1.0e-8 m, already frozen) and `sign(dL)` contradicts the frozen anchor,
    /// where `dL` is taken with `R` the ARITHMETIC MEAN of the two endpoint rows
    /// and `dq` the difference of the two stored coordinate values, on the
    /// WRAP-ON columns. Under a POINTWISE per-pose convention the same four
    /// anchors conflict but the reported angles become 130/140/150 and 135/140
    /// deg; an unpinned convention would make this gate go red on a refactor
    /// instead of on a model change, which is the exact failure this battery's
    /// conversion doctrine exists to prevent.
    /// THE REGISTER, enumerated on 2026-08-21 BEFORE the clause is implemented,
    /// re-derived by hand from `opensim_moment_arms.txt` (+ the FD sidecar) and
    /// EXACTLY:
    ///     elbow_flexion / TRIlong_r   midpoints 135.0 and 145.0 deg
    ///     elbow_flexion / TRImed_r    midpoints 135.0 and 145.0 deg
    ///     elbow_flexion / TRIlat_r    midpoints 135.0 and 145.0 deg
    ///     knee_flexion  / bfsh140_r   midpoint  137.5 deg
    ///     all 22 other anchors: EMPTY over the FULL oracle range, INCLUDING the
    ///     ankle family's −40..+30 deg oracle range, which is WIDER than
    ///     G4(a)'s own registered −30..+20 (the 8 extension-only pairs per ankle
    ///     anchor, 32 cells, are all correct-signed with |dL| ≥ 1.3096463e-3 m).
    /// 4 conflicting anchors, 7 conflict cells, 22 clean. Every cell is present
    /// and sign-agreeing in every column that exists for it; the analytic column
    /// and the stored-length column differ in MAGNITUDE by 0.596-2.023 % at the
    /// three elbow anchors and 6.334 % at the knee anchor, taken RELATIVE TO THE
    /// STORED-LENGTH COLUMN (the same gaps read 0.592-1.983 % and 6.762 %
    /// relative to the analytic one — the normalisation is named so the figure
    /// is checkable rather than quotable).
    /// COLUMN AVAILABILITY, stated because "both independent columns" is FALSE as
    /// prose: there are THREE candidate columns and the third does not exist for
    /// four anchors. `opensim_moment_arms_fd.txt` declares `muscles 66` against
    /// the analytic fixture's `muscles 104`, and `bflh140_r`, `sart_r`,
    /// `soleus_r` and `tibant_r` carry no PathWrap and are absent from it
    /// entirely. The registered column PAIR is therefore
    /// {analytic WrapOn moment arm, adjacent difference of stored WrapOn length},
    /// which exists for all 26. OpenSim's own central difference is printed as a
    /// THIRD, INFORMATIONAL column and must print UNAVAILABLE-BY-CONSTRUCTION for
    /// those four — never a pass, and never a "fixture defect" alarm on an anchor
    /// that structurally has only one comparable column.
    /// Bars: (i) the measured register equals the enumerated set EXACTLY — a cell
    /// that APPEARS or DISAPPEARS is RED, because either the shipped model
    /// changed or the fixture did; (ii) both registered columns are computed and
    /// printed, the analytic one authoritative for muscles carrying ≥ 2
    /// PathWraps and the stored length for the rest, and a cell visible in only
    /// one of THOSE TWO is a fixture defect to investigate, never a model finding
    /// to register; (iii) every muscle in the register must be OFF the Rule-0
    /// displayed set, OR the layer must return `MuscleLengthMode.indeterminate`
    /// for that muscle whenever the conflicted coordinate lies inside its
    /// conflicted interval.
    /// ⚠️ **EXPECTED VERDICT, DECLARED BEFORE THE RUN (adversarial review,
    /// 2026-08-21): G4(g) IS KNOWN-PASS IN ITS ENTIRETY TODAY. It is a REGRESSION
    /// PIN in this battery's measured-outcome-pin idiom, NOT an open
    /// measurement.** Two independent re-derivations (V1's and the reviewer's own
    /// parser) reproduced the enumerated 4 anchors / 7 cells / 22 clean before a
    /// line of it was implemented; every cell is present and sign-agreeing in
    /// both registered columns; and (iii)'s off-surface branch passes because no
    /// capsule `MuscleOverlay` declares is a triceps or a `bfsh`. Its VALUE is
    /// forward-looking — it goes RED if `FullBody.osim`'s geometry, the oracle
    /// fixture, or the Rule-0 displayed set moves — and a first green reading may
    /// NEVER be quoted as evidence that anything was discovered. Recording this
    /// before the run is the point: a clause whose outcome is knowable in advance
    /// and is not SAID to be is the "test that would have passed against the
    /// broken implementation" this document's process notes warn about.
    /// ⚠️ **CLAUSE (iii) IS PART REAL AND PART VACUOUS TODAY, and the split is
    /// declared before the run.** Its off-surface branch is scored on a
    /// population of 4 and CAN go red (it does the moment the register or
    /// `MuscleOverlay.muscleDefs` moves). Its abstention branch is scored on a
    /// population of ZERO, because all four conflicted muscles are off the
    /// product surface today, so that branch is 0 == 0 BY CONSTRUCTION: it must
    /// print VACUOUS-BY-CONSTRUCTION and MUST NOT be counted as a pass, quoted as
    /// evidence, or reported as "abstention is implemented".
    /// NON-VACUITY OF THE REGISTER ITSELF, measured on 2026-08-21 rather than
    /// hoped: all 26 anchors clear the 1.0e-8 m deadband on 100 % of their pairs
    /// (637 of 637), so no cell is decided by an empty population; the register is
    /// BYTE-IDENTICAL at every deadband from 0 through 1.5e-4 m, so the 1.0e-8
    /// constant is decoration here and not the discriminator; the smallest
    /// correct-signed |dL| anywhere is 1.9339269e-4 m (`bfsh140_r`, midpoint
    /// 132.5 deg), ≥ 19,000× the deadband; and INVERTING all 26 registered
    /// directions flips 26 of 26 anchors to conflicting (630 cells), so this is a
    /// control that CAN fail.
    /// WHY IT IS A MODEL PROPERTY AND NOT A CONFIGURATION ARTEFACT: this
    /// oracle-derived register agrees in MEMBERSHIP and BAND with G4(a)'s own
    /// pins (:1193-1198, measured on `neutral`-based constructed poses at a 5 deg
    /// step) even though the oracle's elbow base carries `shoulder_elv_r = 30
    /// deg` and its knee base `hip_flexion_r = 20 deg`. Agreement across a
    /// base-pose change makes the reversal a property of `FullBody.osim`.
    /// WITHDRAWAL / FALSIFICATION: if the register ever grows to include a
    /// DISPLAYED muscle, this layer does not ship until either the model geometry
    /// is repaired or (iii)'s abstention is implemented AND measured on a
    /// non-empty population. If the elbow wrap geometry is later repaired so the
    /// reversal disappears, (i) goes RED and the whole successor is
    /// re-adjudicated rather than silently passing.
    ///
    /// ─── NAVIGATIONAL ADDENDUM, added 2026-08-22 when the two successor
    /// clauses became EXECUTABLE. Nothing above is edited: no registered
    /// sentence, no bar, no enumerated cell, no expected verdict. This note only
    /// says WHERE the text above now runs, because the clauses it registers no
    /// longer govern the method they sit on. ───
    /// G4(f) executes in `testG4fPortReproducesTheShippedModelOnTheOracleSweeps`
    /// and G4(g) in `testG4gTheModelAnchorConflictRegisterIsExactlyAsEnumerated`,
    /// immediately below this method. G4(a)'s own verdict, pins and population
    /// are untouched by both.
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

    // MARK: - G4(f) + G4(g): the two premises G4(a) could not fail separately

    /// One adjacent ORACLE sweep pair, scored for one registered G4 anchor.
    ///
    /// PURE ORACLE CONTENT: no port value is stored here, which is exactly what
    /// lets G4(g) claim that no change to the port can move it. G4(f) computes
    /// its port column separately, in `g4PortFidelity`.
    private struct G4OracleCell {
        let sweep: String
        let muscle: String
        let coordinate: String
        let poseA: Int
        let poseB: Int
        /// PAIR MIDPOINT of the swept coordinate, degrees — the PINNED
        /// convention. Under a pointwise per-pose convention the same four
        /// anchors conflict at different angles, which is why the convention is
        /// pinned rather than assumed.
        let midpointDegrees: Double
        let poseADegrees: Double
        let poseBDegrees: Double
        /// `-0.5*(R_on(a) + R_on(b))*dq` — the ANALYTIC WrapOn column at the
        /// pair midpoint. The column the G4(g) definition pins.
        let analyticDeltaLength: Double
        /// `lengthWrapOn(b) - lengthWrapOn(a)` — the STORED-LENGTH WrapOn column.
        let storedDeltaLength: Double
        /// `-0.5*(r_fd(a) + r_fd(b))*dq` from `opensim_moment_arms_fd.txt`, the
        /// THIRD informational column. `nil` = UNAVAILABLE-BY-CONSTRUCTION: the
        /// muscle carries no PathWrap and that fixture does not declare it.
        let finiteDifferenceDeltaLength: Double?
        /// The WRAP-OFF analytic column. Computed only so the gate can PRINT the
        /// register a reader would get by substituting it — the registration
        /// forbids the substitution and this is what makes the ban checkable.
        let wrapOffDeltaLength: Double
        /// Pointwise `-R_on(pose)*dq` at each endpoint: the convention the
        /// registration explicitly did NOT choose.
        let pointwiseAtA: Double
        let pointwiseAtB: Double
        /// `+1` when the frozen anchor says LENGTHENING, `-1` for SHORTENING.
        let expectedSign: Int
        let pathWraps: Int
        /// Analytic for muscles carrying >= 2 PathWraps, stored length for the
        /// rest — the same multi-wrap split G1 already registers.
        var authoritativeDeltaLength: Double {
            pathWraps >= 2 ? analyticDeltaLength : storedDeltaLength
        }
        var anchor: String { "\(sweep)/\(muscle)" }
        var registerKey: String { String(format: "%@@%.1f", anchor, midpointDegrees) }
    }

    /// The FD sidecar, loaded once. Same idiom as `sharedContext`: a failed load
    /// fails every dependent gate with the same diagnostic instead of going
    /// vacuous.
    private static let sharedFiniteDifference: Result<OpenSimFiniteDifferenceFixture.Table, Error> = {
        Result {
            try OpenSimFiniteDifferenceFixture.load(bundle: Bundle(for: MuscleLengthModeTests.self))
        }
    }()

    /// `true` when `deltaLength` clears the deadband AND contradicts the frozen
    /// anchor. The default deadband is the frozen quantisation floor, which is
    /// the value the G4(g) definition names.
    private static func g4Conflicts(
        _ deltaLength: Double, expectedSign: Int,
        deadband: Double = MuscleLengthModeClassifier.lengthQuantisationFloorMetres
    ) -> Bool {
        guard abs(deltaLength) > deadband else { return false }
        return (deltaLength > 0 ? 1 : -1) != expectedSign
    }

    /// Every registered anchor x every adjacent pair of the ORACLE'S OWN sweep
    /// poses for that anchor's coordinate: 637 cells by construction
    /// (10*28 knee + 4*28 ankle + 5*28 hip + 7*15 elbow).
    private func g4OracleCells(ctx: ModelContext) throws -> [G4OracleCell] {
        let table = ctx.table
        let fd = try Self.sharedFiniteDifference.get()
        let allPairs = sweepPairs(table)
        XCTAssertEqual(allPairs.count, 111, "the registered single-DOF sweep population")
        let toDegrees = 180.0 / Double.pi
        var out: [G4OracleCell] = []

        for sweep in Self.g4Sweeps {
            let pairs = allPairs.filter { $0.coordinate == sweep.coordinate }
            for muscle in sweep.lengthening + sweep.shortening {
                let expected = sweep.lengthening.contains(muscle) ? 1 : -1
                guard let m = table.muscleIndex(muscle),
                      let k = table.muscles[m].coordinates.firstIndex(of: sweep.coordinate),
                      let j = table.coordinateNames.firstIndex(of: sweep.coordinate) else {
                    XCTFail("G4(f)/(g): \(muscle) does not span \(sweep.coordinate) in the oracle")
                    continue
                }
                let wraps = ctx.computer.pathWrapCount(forMuscleNamed: muscle)
                let fdMuscle = fd.muscleIndex(muscle)
                let fdCoordinate = fdMuscle.flatMap {
                    fd.muscles[$0].coordinates.firstIndex(of: sweep.coordinate)
                }

                for pair in pairs {
                    guard let rowA = table.row(pose: pair.a, muscle: m),
                          let rowB = table.row(pose: pair.b, muscle: m),
                          k < rowA.momentArmsWrapOn.count, k < rowB.momentArmsWrapOn.count,
                          k < rowA.momentArmsWrapOff.count, k < rowB.momentArmsWrapOff.count else {
                        XCTFail("G4(f)/(g): the oracle has no row for \(muscle) "
                                + "at pair \(pair.a)->\(pair.b)")
                        continue
                    }
                    let qa = table.poses[pair.a].values[j]
                    let qb = table.poses[pair.b].values[j]
                    let dq = qb - qa

                    var fdDelta: Double?
                    if let fi = fdMuscle, let fk = fdCoordinate,
                       let fdPoseA = fd.poseIndex(table.poses[pair.a].id),
                       let fdPoseB = fd.poseIndex(table.poses[pair.b].id),
                       let fdRowA = fd.row(pose: fdPoseA, muscle: fi),
                       let fdRowB = fd.row(pose: fdPoseB, muscle: fi),
                       fk < fdRowA.momentArms.count, fk < fdRowB.momentArms.count {
                        fdDelta = -0.5 * (fdRowA.momentArms[fk] + fdRowB.momentArms[fk]) * dq
                    }

                    out.append(G4OracleCell(
                        sweep: sweep.name, muscle: muscle, coordinate: sweep.coordinate,
                        poseA: pair.a, poseB: pair.b,
                        midpointDegrees: 0.5 * (qa + qb) * toDegrees,
                        poseADegrees: qa * toDegrees,
                        poseBDegrees: qb * toDegrees,
                        analyticDeltaLength:
                            -0.5 * (rowA.momentArmsWrapOn[k] + rowB.momentArmsWrapOn[k]) * dq,
                        storedDeltaLength: rowB.lengthWrapOn - rowA.lengthWrapOn,
                        finiteDifferenceDeltaLength: fdDelta,
                        wrapOffDeltaLength:
                            -0.5 * (rowA.momentArmsWrapOff[k] + rowB.momentArmsWrapOff[k]) * dq,
                        pointwiseAtA: -rowA.momentArmsWrapOn[k] * dq,
                        pointwiseAtB: -rowB.momentArmsWrapOn[k] * dq,
                        expectedSign: expected, pathWraps: wraps))
                }
            }
        }
        return out
    }

    private struct G4PortFidelityResult {
        let sweep: String
        let muscle: String
        let pathWraps: Int
        /// Which WRAP-ON column carried the MODEL's direction for this anchor.
        let referenceColumn: String
        let spanning: Int
        let scored: Int
        let agreements: Int
        let strongCells: Int
        let strongDisagreements: Int
        /// Reported, not gated: WHERE the port left the model, in degrees of the
        /// swept coordinate at the pair midpoint.
        let disagreeingMidpointsDegrees: [Double]
        var agreement: Double { scored == 0 ? 0 : Double(agreements) / Double(scored) }
        var coverage: Double { spanning == 0 ? 0 : Double(scored) / Double(spanning) }
        var anchor: String { "\(sweep)/\(muscle)" }
    }

    /// The PORT evaluated at the ORACLE's own sweep midpoints, scored against
    /// the MODEL's own WrapOn direction cell by cell. The port enters ONLY here.
    private func g4PortFidelity(ctx: ModelContext) throws -> [G4PortFidelityResult] {
        let table = ctx.table
        let allPairs = sweepPairs(table)
        var out: [G4PortFidelityResult] = []

        for sweep in Self.g4Sweeps {
            let anchors = sweep.lengthening + sweep.shortening
            let present = anchors.filter { ctx.muscleIndexByName[$0] != nil }
            XCTAssertEqual(present.count, anchors.count,
                           "a G4 anchor is not in the parsed model: "
                           + "\(Set(anchors).subtracting(present))")
            guard !present.isEmpty else { continue }

            let pairs = allPairs.filter { $0.coordinate == sweep.coordinate }
            let coordinates = spanUnion(present, table: table)
            let noise = fixtureNoise(coordinates.count)

            var spanning = [Int](repeating: 0, count: present.count)
            var scored = [Int](repeating: 0, count: present.count)
            var agreements = [Int](repeating: 0, count: present.count)
            var strong = [Int](repeating: 0, count: present.count)
            var strongDisagree = [Int](repeating: 0, count: present.count)
            var disagreements = [[Double]](repeating: [], count: present.count)

            for pair in pairs {
                // Same re-indexing law as G1: the fixture's coordinate order and
                // the LIVE model's DOF order differ, so the pose is rebuilt by
                // NAME before it reaches the solver.
                let qa = table.poses[pair.a].values
                let qb = table.poses[pair.b].values
                let poseA = Self.orderedPose(ctx: ctx, table: table, poseIndex: pair.a)
                let poseB = Self.orderedPose(ctx: ctx, table: table, poseIndex: pair.b)
                let mid = zip(poseA, poseB).map { 0.5 * ($0 + $1) }
                guard let rows = ctx.momentArms(pose: mid, muscles: present,
                                                coordinates: coordinates) else {
                    XCTFail("G4(f): moment arms failed at sweep pair \(pair.a)->\(pair.b)")
                    continue
                }
                let dq = coordinates.map { name -> Double in
                    guard let j = table.coordinateNames.firstIndex(of: name) else { return 0 }
                    return qb[j] - qa[j]
                }

                for (m, name) in present.enumerated() {
                    guard let index = table.muscleIndex(name),
                          let k = table.muscles[index].coordinates.firstIndex(of: pair.coordinate),
                          let j = table.coordinateNames.firstIndex(of: pair.coordinate),
                          let rowA = table.row(pose: pair.a, muscle: index),
                          let rowB = table.row(pose: pair.b, muscle: index),
                          k < rowA.momentArmsWrapOn.count, k < rowB.momentArmsWrapOn.count
                    else { continue }
                    spanning[m] += 1

                    let deadband = MuscleLengthModeClassifier.stepDeadbandMetres(
                        momentArmRow: rows[m], jointNoiseRadians: noise,
                        velocityNoiseGain: Self.velocityGain)

                    // The MODEL's own direction on the REGISTERED column pair,
                    // WRAP-ON. The WrapOff columns are NOT the reference here.
                    let reference: Double
                    if ctx.computer.pathWrapCount(forMuscleNamed: name) >= 2 {
                        reference = -0.5 * (rowA.momentArmsWrapOn[k] + rowB.momentArmsWrapOn[k])
                            * (qb[j] - qa[j])
                    } else {
                        reference = rowB.lengthWrapOn - rowA.lengthWrapOn
                    }

                    guard abs(reference) > deadband else { continue }
                    scored[m] += 1
                    let ours = MuscleLengthModeClassifier.lengthRate(momentArmRow: rows[m],
                                                                     jointVelocity: dq)
                    let agrees = (ours > 0) == (reference > 0)
                    if agrees {
                        agreements[m] += 1
                    } else {
                        disagreements[m].append(0.5 * (qa[j] + qb[j]) * 180.0 / .pi)
                    }
                    if abs(reference) >= 10.0 * deadband {
                        strong[m] += 1
                        if !agrees { strongDisagree[m] += 1 }
                    }
                }
            }

            for (m, name) in present.enumerated() {
                let wraps = ctx.computer.pathWrapCount(forMuscleNamed: name)
                out.append(G4PortFidelityResult(
                    sweep: sweep.name, muscle: name, pathWraps: wraps,
                    referenceColumn: wraps >= 2 ? "analytic_wrap_on" : "stored_length_wrap_on",
                    spanning: spanning[m], scored: scored[m], agreements: agreements[m],
                    strongCells: strong[m], strongDisagreements: strongDisagree[m],
                    disagreeingMidpointsDegrees: disagreements[m].sorted()))
            }
        }
        return out
    }

    /// G4(f) PORT FIDELITY, FULL 26-ANCHOR SET.
    ///
    /// The clause — its population, its grids, its WRAP-ON column pair, its
    /// PAIR-MIDPOINT convention, its three bars and its ZERO-TOLERANCE
    /// disclosure — is registered VERBATIM in the successor pre-registration
    /// above `testG4PhysiologyDirectionsOnConstructedSweeps` and is NOT restated
    /// here. This method only executes it.
    ///
    /// WHAT IT ASSERTS: that the PORT's direction agrees with the SHIPPED
    /// MODEL's own direction. NOT one word about anatomy — that is G4(g). It
    /// cannot be repaired by editing an anchor, because the anchor only supplies
    /// the population; the comparison is port-vs-model on both sides.
    ///
    /// GENUINELY UNDETERMINED ON THIS RUN, and that is the point of it. Unlike
    /// G4(g) no re-derivation of this gate exists, nothing was measured before
    /// it was written, and its first reading is a MEASUREMENT rather than a
    /// regression pin. The bars are asserted as registered: the 99.0 % bar is
    /// arithmetically ZERO-TOLERANCE at these population sizes (28 pairs on the
    /// knee/hip/ankle anchors, 15 on the elbow: 27/28 = 96.43 % and 14/15 =
    /// 93.33 % both MISS it), NO minimum-pairs power floor is registered, and
    /// none may be added after a miss.
    func testG4fPortReproducesTheShippedModelOnTheOracleSweeps() throws {
        let ctx = try context()
        let results = try g4PortFidelity(ctx: ctx)

        for r in results {
            var line = "MODE-METRIC g4f sweep=\(r.sweep) muscle=\(r.muscle)"
            line += " path_wraps=\(r.pathWraps) reference_column=\(r.referenceColumn)"
            line += " spanning=\(r.spanning) scored=\(r.scored) agree=\(r.agreements)"
            line += String(format: " agreement=%.6f", r.agreement)
            line += String(format: " coverage=%.6f", r.coverage)
            line += " strong=\(r.strongCells) strong_disagree=\(r.strongDisagreements)"
            let disagreeAt = r.disagreeingMidpointsDegrees.map { String(format: "%.1f", $0) }
            line += " disagree_at_deg=[" + disagreeAt.joined(separator: ",") + "]"
            print(line)
        }

        // The REGISTERED CONSTRUCTION, asserted before any bar is read.
        XCTAssertEqual(results.count, 26, "the G4(f) anchor population moved off its registration")
        XCTAssertEqual(Set(results.map(\.anchor)).count, 26, "two G4(f) anchors collided on one key")
        let spanningBySweep = Dictionary(grouping: results, by: \.sweep)
            .mapValues { group in group.reduce(0) { $0 + $1.spanning } }
        XCTAssertEqual(spanningBySweep["knee_flexion"] ?? -1, 280, "knee: 10 anchors x 28 pairs")
        XCTAssertEqual(spanningBySweep["ankle_dorsiflexion"] ?? -1, 112, "ankle: 4 anchors x 28 pairs")
        XCTAssertEqual(spanningBySweep["hip_flexion"] ?? -1, 140, "hip: 5 anchors x 28 pairs")
        XCTAssertEqual(spanningBySweep["elbow_flexion"] ?? -1, 105, "elbow: 7 anchors x 15 pairs")
        let spanningTotal = results.reduce(0) { $0 + $1.spanning }
        XCTAssertEqual(spanningTotal, 637,
                       "10*28 + 4*28 + 5*28 + 7*15 = 637 scored anchor-pairs")
        // The shoulder sweep EXISTS in the oracle and carries no registered
        // anchor, so it must contribute nothing — the frozen exclusion, executed.
        XCTAssertEqual(sweepPairs(ctx.table).filter { $0.coordinate == "shoulder_elv_r" }.count, 12,
                       "the oracle's shoulder sweep")
        XCTAssertTrue(results.allSatisfy { $0.sweep != "shoulder_flexion" },
                      "the shoulder sweep stays excluded, exactly as frozen")

        // The MEASURED outcome, printed as a receipt BEFORE it is adjudicated,
        // so a red run still leaves a complete reading.
        let scoredTotal = results.reduce(0) { $0 + $1.scored }
        let strongTotal = results.reduce(0) { $0 + $1.strongCells }
        let unscored = results.filter { $0.scored == 0 }.map(\.anchor).sorted()
        let worst = results.map(\.agreement).min() ?? 0
        let worstAnchor = results.min(by: { $0.agreement < $1.agreement })?.anchor ?? "none"
        let worstCoverage = results.map(\.coverage).min() ?? 0
        let strongDisagreeTotal = results.reduce(0) { $0 + $1.strongDisagreements }
        let failing = results.filter {
            $0.scored == 0 || $0.agreement < 0.99 || $0.strongDisagreements > 0
                || $0.coverage < 0.60
        }.map(\.anchor).sorted()
        let outcome = failing.isEmpty ? "PASS_NON_VACUOUS" : "FAILED_AGAINST_REGISTERED_BAR"
        var verdict = "MODE-VERDICT gate=G4(f) outcome=\(outcome) anchors=\(results.count)"
        verdict += " spanning=\(spanningTotal) scored=\(scoredTotal) strong=\(strongTotal)"
        verdict += " strong_disagree=\(strongDisagreeTotal) unscored_anchors=\(unscored)"
        verdict += String(format: " worst_agreement=%.6f", worst)
        verdict += String(format: " worst_coverage=%.6f", worstCoverage)
        verdict += " worst_anchor=\(worstAnchor) failing=\(failing)"
        verdict += " registered_bar=[>= 0.99 port-vs-model agreement per anchor; 0 disagreements"
        verdict += " at >= 10x the step deadband; coverage >= 0.60; every anchor scores > 0 pairs]"
        verdict += " zero_tolerance=[28-pair anchors: 27/28 = 0.9643 MISSES 0.99; 15-pair elbow:"
        verdict += " 14/15 = 0.9333 MISSES 0.99; no minimum-pairs power floor is registered]"
        verdict += " claims=[port reproduces the shipped model]"
        verdict += " claims_not=[anatomy — that is G4(g)]"
        verdict += " first_measurement=true provenance=opensim_moment_arms.txt"
        print(verdict)
        if !failing.isEmpty {
            recordFailedGate("G4(f)",
                             measured: String(format: "worst=%.6f on %@; failing anchors=",
                                              worst, worstAnchor)
                                       + "\(failing); strong_disagree=\(strongDisagreeTotal)",
                             bar: ">= 0.99 per anchor, 0 strong disagreements, coverage >= 0.60",
                             why: "the PORT left the SHIPPED MODEL on the oracle's own sweep "
                                  + "poses — this is a port defect, not a registration question, "
                                  + "and it cannot be repaired by editing an anchor")
        }

        // The registered bars, asserted per anchor so a failure NAMES one.
        for r in results {
            XCTAssertGreaterThan(r.scored, 0,
                "G4(f) NON-VACUITY: \(r.anchor) scored no pairs — RED, not skipped")
            XCTAssertGreaterThanOrEqual(r.agreement, 0.99,
                "G4(f): port-vs-model direction agreement on \(r.anchor) is below the 99.0 % bar")
            XCTAssertEqual(r.strongDisagreements, 0,
                "G4(f): a port-vs-model disagreement at >= 10x the step deadband on \(r.anchor)")
            XCTAssertGreaterThanOrEqual(r.coverage, 0.60,
                "G4(f): coverage floor on \(r.anchor)")
        }
    }

    /// G4(g) MODEL-ANCHOR CONFLICT REGISTER.
    ///
    /// The clause — its pinned convention, its enumerated register, its three
    /// bars, its column-availability rule, and its two pre-declared warnings —
    /// is registered VERBATIM in the successor pre-registration above
    /// `testG4PhysiologyDirectionsOnConstructedSweeps` and is NOT restated here.
    ///
    /// DERIVED FROM THE ORACLE ALONE: the port is not an input to this method,
    /// so no change to the port can move it. `ctx` is opened only for the model
    /// text (`pathWrapCount`, the multi-wrap split) and for the Rule-0 displayed
    /// set that clause (iii) reads.
    ///
    /// KNOWN-PASS, DECLARED BEFORE THE RUN AND KEPT: this is a REGRESSION PIN in
    /// this battery's measured-outcome-pin idiom, NOT an open measurement. Two
    /// independent re-derivations reproduced the enumerated 4 anchors / 7 cells /
    /// 22 clean before a line of it was implemented, and a third — this
    /// implementer's, 2026-08-22, `/tmp/g4verify/register.py` against
    /// `BioMotionTests/Fixtures/opensim_moment_arms.txt` — reproduced it again
    /// before this method was written. Its VALUE is forward-looking: it goes RED
    /// if `FullBody.osim`'s geometry, the oracle fixture, or the Rule-0 displayed
    /// set moves. A first green reading may NEVER be quoted as evidence that
    /// anything was discovered.
    ///
    /// CLAUSE (iii) IS PART REAL AND PART VACUOUS: its off-surface branch is
    /// scored on a population of 4 and CAN go red; its abstention branch is
    /// scored on a population of ZERO and prints VACUOUS-BY-CONSTRUCTION.
    func testG4gTheModelAnchorConflictRegisterIsExactlyAsEnumerated() throws {
        let ctx = try context()
        let cells = try g4OracleCells(ctx: ctx)
        let floor = MuscleLengthModeClassifier.lengthQuantisationFloorMetres
        XCTAssertEqual(floor, 1.0e-8, "the frozen quantisation floor moved")

        // The REGISTERED CONSTRUCTION.
        XCTAssertEqual(cells.count, 637,
                       "the G4(g) cell population moved off its registered construction")
        XCTAssertEqual(Set(cells.map(\.anchor)).count, 26, "the 26-anchor set moved")
        let cellsBySweep = Dictionary(grouping: cells, by: \.sweep).mapValues(\.count)
        XCTAssertEqual(cellsBySweep["knee_flexion"] ?? -1, 280, "knee: 10 anchors x 28 pairs")
        XCTAssertEqual(cellsBySweep["ankle_dorsiflexion"] ?? -1, 112, "ankle: 4 anchors x 28 pairs")
        XCTAssertEqual(cellsBySweep["hip_flexion"] ?? -1, 140, "hip: 5 anchors x 28 pairs")
        XCTAssertEqual(cellsBySweep["elbow_flexion"] ?? -1, 105, "elbow: 7 anchors x 15 pairs")

        // THE REGISTER, as enumerated on 2026-08-21 BEFORE implementation.
        let enumeratedRegister: Set<String> = [
            "elbow_flexion/TRIlong_r@135.0", "elbow_flexion/TRIlong_r@145.0",
            "elbow_flexion/TRImed_r@135.0", "elbow_flexion/TRImed_r@145.0",
            "elbow_flexion/TRIlat_r@135.0", "elbow_flexion/TRIlat_r@145.0",
            "knee_flexion/bfsh140_r@137.5",
        ]
        let analyticRegister = Set(cells.filter {
            Self.g4Conflicts($0.analyticDeltaLength, expectedSign: $0.expectedSign)
        }.map(\.registerKey))
        let storedRegister = Set(cells.filter {
            Self.g4Conflicts($0.storedDeltaLength, expectedSign: $0.expectedSign)
        }.map(\.registerKey))
        let authoritativeRegister = Set(cells.filter {
            Self.g4Conflicts($0.authoritativeDeltaLength, expectedSign: $0.expectedSign)
        }.map(\.registerKey))
        let conflictAnchors = Set(cells.filter { analyticRegister.contains($0.registerKey) }
            .map(\.anchor))
        let conflictMuscles = Set(cells.filter { analyticRegister.contains($0.registerKey) }
            .map(\.muscle))

        // Per-anchor receipts, including the THIRD informational column.
        let unavailableByConstruction = Set(cells.filter { $0.finiteDifferenceDeltaLength == nil }
            .map(\.muscle))
        for anchor in Set(cells.map(\.anchor)).sorted() {
            let group = cells.filter { $0.anchor == anchor }
            let fdAvailable = group.allSatisfy { $0.finiteDifferenceDeltaLength != nil }
            let fdConflicts = group.filter { cell -> Bool in
                guard let value = cell.finiteDifferenceDeltaLength else { return false }
                return Self.g4Conflicts(value, expectedSign: cell.expectedSign)
            }.count
            let conflictAngles = group.filter { analyticRegister.contains($0.registerKey) }
                .map { String(format: "%.1f", $0.midpointDegrees) }.sorted()
            let nAnalytic = group.filter { analyticRegister.contains($0.registerKey) }.count
            let nStored = group.filter { storedRegister.contains($0.registerKey) }.count
            let nAuthoritative = group.filter { authoritativeRegister.contains($0.registerKey) }.count
            let nWrapOff = group.filter {
                Self.g4Conflicts($0.wrapOffDeltaLength, expectedSign: $0.expectedSign)
            }.count
            let minAnalyticHere: Double = group.map { abs($0.analyticDeltaLength) }.min() ?? 0
            let minStoredHere: Double = group.map { abs($0.storedDeltaLength) }.min() ?? 0
            var line = "MODE-METRIC g4g anchor=\(anchor) cells=\(group.count)"
            line += " path_wraps=\(group.first?.pathWraps ?? -1)"
            line += " expected_sign=\(group.first?.expectedSign ?? 0)"
            line += " conflicts_analytic=\(nAnalytic)"
            line += " conflicts_stored=\(nStored)"
            line += " conflicts_authoritative=\(nAuthoritative)"
            line += " conflict_midpoints_deg=[" + conflictAngles.joined(separator: ",") + "]"
            line += fdAvailable
                ? " fd_third_column=available conflicts_fd=\(fdConflicts)"
                : " fd_third_column=UNAVAILABLE-BY-CONSTRUCTION conflicts_fd=n/a"
            line += " wrap_off_conflicts=\(nWrapOff)"
            line += String(format: " min_abs_dl_analytic=%.7e", minAnalyticHere)
            line += String(format: " min_abs_dl_stored=%.7e", minStoredHere)
            print(line)
        }

        // (i) THE REGISTER EQUALS THE ENUMERATED SET EXACTLY.
        XCTAssertEqual(analyticRegister, enumeratedRegister,
            "G4(g)(i): the measured register is not the enumerated one — a cell that APPEARS or "
            + "DISAPPEARS is RED, because either the shipped model changed or the fixture did")
        XCTAssertEqual(analyticRegister.count, 7, "G4(g)(i): 7 conflict cells")
        XCTAssertEqual(conflictAnchors.count, 4, "G4(g)(i): 4 conflicting anchors")
        XCTAssertEqual(26 - conflictAnchors.count, 22, "G4(g)(i): 22 clean anchors")
        XCTAssertEqual(conflictMuscles.sorted(),
                       ["TRIlat_r", "TRIlong_r", "TRImed_r", "bfsh140_r"],
                       "G4(g)(i): the conflicting muscle set moved")

        // (ii) BOTH REGISTERED COLUMNS, and the fixture-defect rule.
        XCTAssertEqual(storedRegister, analyticRegister,
            "G4(g)(ii): a conflict cell is visible in only ONE of the two registered columns "
            + "{analytic WrapOn moment arm, adjacent difference of stored WrapOn length} — that "
            + "is a FIXTURE DEFECT to investigate, never a model finding to register")
        XCTAssertEqual(authoritativeRegister, enumeratedRegister,
            "G4(g)(ii): the authoritative split (analytic for >= 2 PathWraps, stored length for "
            + "the rest) does not reproduce the enumerated register")

        // The THIRD column is INFORMATIONAL and structurally absent for four
        // anchors. Asserted as a STRUCTURE, never as a pass.
        XCTAssertEqual(unavailableByConstruction, ["bflh140_r", "sart_r", "soleus_r", "tibant_r"],
            "G4(g): OpenSim's own central difference is unavailable for exactly the four anchors "
            + "that carry no PathWrap and are absent from opensim_moment_arms_fd.txt")
        XCTAssertEqual(cells.filter { $0.finiteDifferenceDeltaLength == nil }.count, 112,
            "G4(g): 4 anchors x 28 pairs are UNAVAILABLE-BY-CONSTRUCTION")
        for cell in cells where unavailableByConstruction.contains(cell.muscle) {
            XCTAssertEqual(cell.pathWraps, 0,
                "G4(g): \(cell.muscle) is missing from the FD fixture but carries a PathWrap — "
                + "that would be a fixture defect, not a structural absence")
        }
        let fdRegister = Set(cells.compactMap { cell -> String? in
            guard let v = cell.finiteDifferenceDeltaLength else { return nil }
            return Self.g4Conflicts(v, expectedSign: cell.expectedSign) ? cell.registerKey : nil
        })
        var thirdColumnLine = "MODE-METRIC g4g third_column=opensim_central_difference"
        thirdColumnLine += " unavailable_anchors=\(unavailableByConstruction.sorted())"
        thirdColumnLine += " unavailable_cells=112 fd_register=\(fdRegister.sorted())"
        thirdColumnLine += " note=INFORMATIONAL_ONLY_never_a_pass_and_never_a_fixture_defect_alarm"
        print(thirdColumnLine)

        // (iii) OFF-SURFACE (population 4, REAL) and ABSTENTION (population 0).
        let displayed = Set(ctx.displayedMuscles)
        let onSurface = conflictMuscles.intersection(displayed)
        XCTAssertEqual(conflictMuscles.count, 4,
            "G4(g)(iii): the off-surface branch must be scored on all 4 conflicted muscles")
        XCTAssertTrue(onSurface.isEmpty,
            "G4(g)(iii): a CONFLICTED muscle reached the Rule-0 displayed set — this layer does "
            + "not ship until either the model geometry is repaired or the abstention branch is "
            + "implemented AND measured on a non-empty population: \(onSurface.sorted())")
        var abstention = "MODE-VERDICT gate=G4(g)(iii)-abstention"
        // Same derivation rule as the gate verdict below: a non-zero population
        // means the abstention branch is REACHABLE and must be measured, not
        // labelled vacuous. Hardcoding the token here would have printed
        // "VACUOUS-BY-CONSTRUCTION population=3".
        abstention += onSurface.isEmpty
            ? " outcome=VACUOUS-BY-CONSTRUCTION population=0"
            : " outcome=REACHABLE_AND_UNMEASURED population=\(onSurface.count)"
        abstention += " conflicted_muscles=\(conflictMuscles.sorted())"
        abstention += " displayed_muscles=\(displayed.count)"
        abstention += " reason=all_four_conflicted_muscles_are_off_the_product_surface_today"
        abstention += " note=0_==_0_scores_nothing__NOT_a_pass"
        abstention += "__NOT_evidence_abstention_is_implemented"
        print(abstention)

        // NON-VACUITY OF THE REGISTER ITSELF — measured, not hoped.
        let clearedAnalytic = cells.filter { abs($0.analyticDeltaLength) > floor }.count
        let clearedStored = cells.filter { abs($0.storedDeltaLength) > floor }.count
        XCTAssertEqual(clearedAnalytic, 637, "G4(g): every cell must clear the 1.0e-8 m deadband")
        XCTAssertEqual(clearedStored, 637, "G4(g): every cell must clear the 1.0e-8 m deadband")
        let minAnalytic = cells.map { abs($0.analyticDeltaLength) }.min() ?? 0
        let minStored = cells.map { abs($0.storedDeltaLength) }.min() ?? 0
        XCTAssertGreaterThan(minAnalytic, 1.5e-4,
            "G4(g): the register is BYTE-IDENTICAL from 0 through 1.5e-4 m only because no cell "
            + "sits inside that band")
        XCTAssertGreaterThan(minStored, 1.5e-4, "G4(g): same, on the stored-length column")
        for deadband in [0.0, 1.0e-8, 1.0e-6, 1.0e-5, 1.0e-4, 1.5e-4] {
            let ladder = Set(cells.filter {
                Self.g4Conflicts($0.analyticDeltaLength, expectedSign: $0.expectedSign,
                                 deadband: deadband)
            }.map(\.registerKey))
            XCTAssertEqual(ladder, enumeratedRegister,
                "G4(g): the register is not byte-identical at deadband \(deadband) — the 1.0e-8 "
                + "constant is decoration here, not the discriminator")
        }

        let clean = cells.filter {
            !Self.g4Conflicts($0.authoritativeDeltaLength, expectedSign: $0.expectedSign)
        }
        XCTAssertEqual(clean.count, 630, "G4(g): 637 cells minus 7 conflict cells")
        let smallestClean = try XCTUnwrap(
            clean.min(by: { abs($0.analyticDeltaLength) < abs($1.analyticDeltaLength) }),
            "G4(g): the clean population is empty")
        XCTAssertEqual(abs(smallestClean.analyticDeltaLength), 1.9339269e-4, accuracy: 1.0e-11,
            "G4(g): the smallest correct-signed |dL| moved off its receipt")
        XCTAssertEqual(smallestClean.muscle, "bfsh140_r", "G4(g): the smallest clean cell moved")
        XCTAssertEqual(smallestClean.midpointDegrees, 132.5, accuracy: 1.0e-6,
                       "G4(g): the smallest clean cell moved off 132.5 deg")
        XCTAssertGreaterThanOrEqual(abs(smallestClean.analyticDeltaLength), 19_000.0 * floor,
            "G4(g): the smallest correct-signed |dL| is >= 19,000x the deadband")
        // DISCLOSURE, printed rather than asserted: the registered 1.9339269e-4
        // figure is the ANALYTIC column's minimum. The STORED column's own
        // minimum is smaller and is printed so the reader can check which column
        // the receipt came from.
        let smallestCleanStored = try XCTUnwrap(
            clean.min(by: { abs($0.storedDeltaLength) < abs($1.storedDeltaLength) }),
            "G4(g): the clean population is empty")
        var smallestLine = "MODE-METRIC g4g smallest_clean_cell"
        smallestLine += String(format: " analytic=%.7e", abs(smallestClean.analyticDeltaLength))
        smallestLine += " analytic_at=\(smallestClean.muscle)"
        smallestLine += String(format: "@%.1f", smallestClean.midpointDegrees)
        smallestLine += String(format: " stored=%.7e", abs(smallestCleanStored.storedDeltaLength))
        smallestLine += " stored_at=\(smallestCleanStored.muscle)"
        smallestLine += String(format: "@%.1f", smallestCleanStored.midpointDegrees)
        smallestLine += " registered_figure=1.9339269e-4 registered_column=analytic"
        smallestLine += " note=the_registered_figure_is_the_ANALYTIC_columns_minimum"
        print(smallestLine)

        // THE CONTROL THAT CAN FAIL: inverting all 26 registered directions.
        let inverted = cells.filter {
            Self.g4Conflicts($0.authoritativeDeltaLength, expectedSign: -$0.expectedSign)
        }
        XCTAssertEqual(inverted.count, 630,
            "G4(g): inverting every registered direction must flip 630 cells to conflicting")
        XCTAssertEqual(Set(inverted.map(\.anchor)).count, 26,
            "G4(g): inverting every registered direction must flip 26 of 26 anchors")

        // THE ANKLE FAMILY'S EXTENSION-ONLY PAIRS: the part of the oracle range
        // that is WIDER than G4(a)'s own registered -30..+20.
        let ankleExtension = cells.filter {
            $0.sweep == "ankle_dorsiflexion"
                && ($0.midpointDegrees < -30.0 || $0.midpointDegrees > 20.0)
        }
        XCTAssertEqual(ankleExtension.count, 32,
            "G4(g): 8 extension-only pairs x 4 ankle anchors")
        XCTAssertTrue(Dictionary(grouping: ankleExtension, by: \.anchor)
                        .values.allSatisfy { $0.count == 8 },
                      "G4(g): 8 extension-only pairs per ankle anchor")
        XCTAssertEqual(ankleExtension.filter {
            Self.g4Conflicts($0.authoritativeDeltaLength, expectedSign: $0.expectedSign)
        }.count, 0, "G4(g): every extension-only ankle cell is correct-signed")
        XCTAssertGreaterThanOrEqual(ankleExtension.map { abs($0.analyticDeltaLength) }.min() ?? 0,
                                    1.3096463e-3,
            "G4(g): the extension-only ankle cells sit at |dL| >= 1.3096463e-3 m")

        // THE CONVENTION PIN, made executable. Under a POINTWISE per-pose
        // convention the SAME four anchors conflict at DIFFERENT angles; pinning
        // the pair-midpoint convention is what keeps this gate red on a MODEL
        // change instead of on a refactor.
        var pointwiseValue: [String: Double] = [:]
        var pointwiseExpected: [String: Int] = [:]
        for cell in cells {
            let keyA = String(format: "%@@%.1f", cell.anchor, cell.poseADegrees)
            let keyB = String(format: "%@@%.1f", cell.anchor, cell.poseBDegrees)
            pointwiseValue[keyA] = cell.pointwiseAtA
            pointwiseValue[keyB] = cell.pointwiseAtB
            pointwiseExpected[keyA] = cell.expectedSign
            pointwiseExpected[keyB] = cell.expectedSign
        }
        let pointwiseRegister = Set(pointwiseValue.keys.filter { key -> Bool in
            guard let value = pointwiseValue[key], let expected = pointwiseExpected[key] else {
                return false
            }
            return Self.g4Conflicts(value, expectedSign: expected)
        })
        let enumeratedPointwise: Set<String> = [
            "elbow_flexion/TRIlong_r@130.0", "elbow_flexion/TRIlong_r@140.0",
            "elbow_flexion/TRIlong_r@150.0",
            "elbow_flexion/TRImed_r@130.0", "elbow_flexion/TRImed_r@140.0",
            "elbow_flexion/TRImed_r@150.0",
            "elbow_flexion/TRIlat_r@130.0", "elbow_flexion/TRIlat_r@140.0",
            "elbow_flexion/TRIlat_r@150.0",
            "knee_flexion/bfsh140_r@135.0", "knee_flexion/bfsh140_r@140.0",
        ]
        XCTAssertEqual(pointwiseRegister, enumeratedPointwise,
            "G4(g): the POINTWISE convention no longer reports 130/140/150 and 135/140 deg — the "
            + "convention is PINNED precisely so this gate goes red on a model change, not a "
            + "refactor")
        XCTAssertEqual(Set(pointwiseRegister.map { $0.components(separatedBy: "@")[0] }),
                       conflictAnchors,
            "G4(g): the two conventions must name the SAME four anchors and differ only in angle")

        // THE WRAP-OFF COLUMNS ARE NOT THE REFERENCE, and this is why.
        let wrapOffRegister = Set(cells.filter {
            Self.g4Conflicts($0.wrapOffDeltaLength, expectedSign: $0.expectedSign)
        }.map(\.registerKey))
        let wrapOffAnchors = Set(cells.filter { wrapOffRegister.contains($0.registerKey) }
            .map(\.anchor))
        XCTAssertEqual(wrapOffAnchors.count, 7,
            "G4(g): substituting the WrapOff columns produces a completely different 7-anchor set")
        XCTAssertTrue(wrapOffAnchors.isDisjoint(with: conflictAnchors),
            "G4(g): the WrapOff conflict set must be DISJOINT from the registered one — an "
            + "implementer reading the wrong field would produce a self-consistent but wrong gate")
        var wrapOffLine = "MODE-METRIC g4g wrap_off_substitution"
        wrapOffLine += " anchors=\(wrapOffAnchors.sorted()) cells=\(wrapOffRegister.count)"
        wrapOffLine += " note=NOT_THE_REFERENCE__printed_only_so_the_ban_is_checkable"
        print(wrapOffLine)

        let cleanAnchorCount = 26 - conflictAnchors.count
        // The outcome TOKEN is DERIVED, never hardcoded. This class sets no
        // `continueAfterFailure`, so it runs at XCTest's default of `true` and a
        // failed assertion above still reaches this print. A hardcoded
        // `PASS_NON_VACUOUS` would therefore emit a PASS receipt on a RED run —
        // and STATUS's round-16 header says in terms that the MODE-VERDICT lines
        // are the reading. Caught by adversarial review 2026-08-22 before this
        // gate had ever run.
        let gRegisterHolds = analyticRegister == enumeratedRegister
            && storedRegister == enumeratedRegister
            && conflictAnchors.count == 4 && analyticRegister.count == 7
            && cleanAnchorCount == 22
            && clearedAnalytic == cells.count
            && onSurface.isEmpty
        let gOutcome = gRegisterHolds
            ? "PASS_NON_VACUOUS" : "FAILED_AGAINST_REGISTERED_BAR"
        if !gRegisterHolds {
            recordFailedGate("G4(g)", measured: "register=\(analyticRegister.sorted()) "
                             + "anchors_conflicting=\(conflictAnchors.count) "
                             + "cells=\(analyticRegister.count) clean=\(cleanAnchorCount) "
                             + "on_surface=\(onSurface.sorted())",
                             bar: "register == the enumerated 4 anchors / 7 cells / 22 clean, "
                                  + "both registered columns agreeing, every conflicted muscle "
                                  + "off the Rule-0 displayed set",
                             why: "the shipped model's geometry, the oracle fixture, or the "
                                  + "displayed set moved — which is the ONLY thing this "
                                  + "regression pin exists to catch")
        }
        var gVerdict = "MODE-VERDICT gate=G4(g) outcome=\(gOutcome)"
        gVerdict += " register=\(analyticRegister.sorted())"
        gVerdict += " anchors_conflicting=\(conflictAnchors.count)"
        gVerdict += " cells_conflicting=\(analyticRegister.count)"
        gVerdict += " anchors_clean=\(cleanAnchorCount) cells_total=\(cells.count)"
        gVerdict += " cleared_deadband=\(clearedAnalytic)/\(cells.count)"
        gVerdict += " inverted_control=[630 cells, 26 of 26 anchors]"
        gVerdict += " registered_bar=[(i) register == enumerated EXACTLY; (ii) both registered"
        gVerdict += " columns agree; (iii) every conflicted muscle OFF the Rule-0 displayed set]"
        gVerdict += " known_pass_declared_before_run=true regression_pin=true"
        gVerdict += " discovery=NONE__a_first_green_reading_may_not_be_quoted_as_evidence"
        gVerdict += " clause_iii_abstention=VACUOUS-BY-CONSTRUCTION"
        gVerdict += " port_is_an_input=false provenance=opensim_moment_arms.txt+fd_sidecar"
        print(gVerdict)
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
    ///
    /// ─── SUCCESSOR PRE-REGISTRATION, written 2026-08-21, RED-FIRST, BEFORE the
    /// measurement that adjudicates it ───
    ///
    /// **G9(b) stands FAILED and is SUPERSEDED-NOT-ERASED BY G9(b2).** The text
    /// above is not edited, the pins are not re-baselined, and
    /// `RegisteredBar.g9RequiredDisagreements` keeps the value 1 and is REUSED
    /// VERBATIM below. The BAR does not move; the PERTURBATION CLASS does.
    ///
    /// THE ALGEBRA THAT FORCES A CLASS CHANGE — the two formulas, verbatim from
    /// the shipped source, and the invariance they imply. Re-derived against
    /// `BioMotion/Muscle/MuscleLengthMode.swift` on 2026-08-21, not quoted (every
    /// bare `MLM:` citation below is a line in THAT file, not in this one):
    ///
    ///     jitterMetres(R, σ)    = sqrt( Σⱼ (R[j]·σ[j])² )      MLM:266-274
    ///     stepDeadbandMetres(R) = max( k·g·jitterMetres(R,σ), F )  MLM:277-282
    ///     lengthRate(R, dq)     = − Σⱼ R[j]·dq[j]               MLM:309-314
    ///     classify(v, D)        = lengthening if v > D;
    ///                             shortening if v < −D; else third  MLM:319-324
    ///
    /// with `k = 3` (MLM:238) and
    /// `F = lengthQuantisationFloorMetres = 1.0e-8 m` (MLM:244).
    /// `jitterMetres` is HOMOGENEOUS OF DEGREE 1 in the moment-arm row and
    /// `lengthRate` is LINEAR in it, and the registered control is a UNIFORM ROW
    /// SCALE `R → (1+ε)R` with ε = +0.01114 > 0. Therefore:
    ///   * in the `k·g·s` branch, `v → (1+ε)v` and `D → (1+ε)D`, so `v > D` and
    ///     `v < −D` are EXACTLY invariant — same mode, at ANY multiplicative
    ///     size;
    ///   * in the `F` floor branch, `D` is unchanged while `|v|` grows, so the
    ///     test can only move THIRD-STATE → DIRECTIONAL, never across the sign;
    ///   * in the branch-crossing case (`k·g·s < F ≤ k·g·(1+ε)s`) the same
    ///     one-way conclusion holds, since `D' = (1+ε)k·g·s ≤ (1+ε)D`.
    /// A positive scalar multiple can therefore NEVER flip lengthening ↔
    /// shortening, at any size, in any parameterisation. G9(b) is not weakly
    /// detectable; its error class is UNREACHABLE. No re-sizing of the old
    /// control repairs that, which is why the successor is a different CLASS and
    /// not a different constant.
    /// THE FIXTURE-FACE NUMBERS, re-derived here rather than cited: σ is frozen
    /// at 1.0e-6 rad (MLM:249), taps = 9 (MLM:257), and
    /// `WindowedDerivativeFilter.velocityNoiseGain(taps: 9)` — the L2 norm of the
    /// order-3 Savitzky-Golay first-derivative coefficients built at
    /// `NimbleEngine.swift:2755-2790`, order from `:2692` — is
    /// 0.33813876244990926, so `k·g = 1.0144162873497278`. `dq` has EXACTLY ONE
    /// non-zero entry, +10 deg = 0.17453292519943295 rad (:1681-1686). The
    /// classifier is therefore a SIGN TEST on the swept moment-arm entry `a`:
    /// directional iff `|a| > 5.812177193446956e-6·‖R‖₂`, or `|a| >
    /// 5.7295779513082324e-8 m` in the floor branch, which binds only when
    /// `‖R‖₂ < 9.857885884429244e-3 m`. 1.114 % of `a` is < 100 % of `a` at every
    /// step, which is the whole story.
    ///
    /// **G9(b2) SIGN-CLASS DISCRIMINATION.** The control NEGATES one leg's
    /// moment-arm row (equivalently its swept entry alone; for this single-DOF
    /// drive the two are observationally identical, and BOTH forms are run so the
    /// equivalence is MEASURED rather than argued). This is the error class the
    /// FROZEN registration itself assigns to G9 — `MuscleLengthMode.swift`'s
    /// "WHAT D DOES NOT CONTAIN" note says a sign flip in `R` is a
    /// full-magnitude sign flip in `dL/dt` that no 3σ pose band absorbs, bounded
    /// "by G1 …, G9 … and G7" — and it is the class the 173-pose cylinder-wrap
    /// receipt actually produced (4 surviving SIGN FLIPS). The 1.114 %
    /// multiplicative form was the mis-registration.
    /// WHY IT CHANGES THE OUTPUT, verified against the source and EXACT in
    /// floating point, not approximate:
    ///   * `jitterMetres` sums SQUARES, and IEEE-754 negation only flips a sign
    ///     bit, so `‖R‖` and hence `D_L` are BIT-IDENTICAL;
    ///   * `lengthRate` is linear and odd and round-to-nearest-even is
    ///     sign-symmetric, so `v_L → −v_L` EXACTLY;
    ///   * hence `|v_L|` is unchanged, `isDirectional` is unchanged at EVERY
    ///     step, and the scored/excluded partition is BYTE-IDENTICAL to the
    ///     unperturbed run's — which repairs BY CONSTRUCTION the second caveat
    ///     that binds the additive companion (its 528 steps are not G9(a)'s 258).
    /// THE PREDICTED COUNT, DERIVED ARITHMETICALLY AND WRITTEN BEFORE THE RUN.
    /// A CORRECTION FIRST, because the prior draft of this derivation had it
    /// wrong: `scored` is `modeR.isDirectional || modeL.isDirectional` — an OR
    /// (:1745-1749), not an AND. The count survives the correction, as follows.
    ///   1. 33 steps × 16 pairs = 528, re-derived from the sweep table at
    ///      :1681-1686 (knee 0→140/10 = 14 steps, hip −20→120/10 = 14, ankle
    ///      −30→20/10 = 5), and 528 = 258 + 270 against the pins at :1584-1585.
    ///   2. A positive multiplicative ε can only move a step THIRD → DIRECTIONAL
    ///      (proved above), so the multiplicative run's scored SET CONTAINS the
    ///      unperturbed one.
    ///   3. Suppose some step is scored only in the multiplicative run. Then
    ///      unperturbed `modeL` is the third state. If `modeR` is directional the
    ///      UNPERTURBED run scores that step by OR and `modeR ≠ modeL`,
    ///      contradicting G9(a)'s pinned 0 disagreements (:1391). If `modeR` is
    ///      the third state, the MULTIPLICATIVE run scores it and `modeR ≠ modeL`
    ///      (now directional), contradicting the pinned 0 at :1586-1587. No such step
    ///      exists, so the UNPERTURBED scored population is EXACTLY 258.
    ///   4. On each of those 258, G9(a) reads `modeR == modeL`; OR-scoring plus
    ///      equality forces BOTH to be the SAME DIRECTIONAL mode.
    ///   5. Under the sign flip `modeL` becomes the OPPOSITE directional mode and
    ///      `modeR` is untouched, so `modeR ≠ modeL` on ALL 258.
    /// PREDICTION, PRE-REGISTERED: disagreements = 258 = scored; excluded = 270.
    /// Bars: (i) disagreements ≥ `RegisteredBar.g9RequiredDisagreements` (1),
    /// unchanged and reused verbatim; (ii) `scored` and `excluded` must equal the
    /// UNPERTURBED run's, asserted against THAT RUN rather than against a
    /// literal, so the population is comparable with G9(a)'s by construction;
    /// (iii) COHERENCE: `disagreements == scored`, and
    /// `disagreementsOnUnperturbedExcludedSteps == 0`.
    /// ⚠️ **ITS OWN ESCAPE HATCHES, named because the count is arithmetically
    /// forced and a forced prediction is at risk of being a control that cannot
    /// fail in the OTHER direction:**
    ///   (h1) Reading exactly 258 confirms only that the theorem's premises still
    ///        hold. This clause's discriminating power lives in its FAILURE
    ///        modes — anything other than `disagreements == scored` is RED — and
    ///        NOT in the match. A match may never be quoted as evidence that the
    ///        mirror check is sensitive to a REALISTIC error; it is evidence
    ///        about the SIGN class only. CONFIRMED BY ADVERSARIAL REVIEW
    ///        2026-08-21: the 258 is arithmetically FORCED by the pins already in
    ///        this file (:1391, :1584-1587) plus the sign algebra, i.e. this
    ///        clause's outcome is determinable from the repo WITHOUT running it.
    ///        That is disclosed here rather than discovered later, and no round
    ///        may cite a 258 reading as new evidence of anything.
    ///   (h2) IT READS 0 DESPITE A REAL DEFECT precisely when the two legs' modes
    ///        are computed from the SAME row — i.e. when the perturbation lands
    ///        UPSTREAM of the left/right split, so `modeR` sees the flipped row
    ///        too and both flip together. That is the left/right aliasing G9(a)
    ///        exists to catch, and G9(a) itself CANNOT see it, because an aliased
    ///        pair agrees trivially. A 0 here therefore means EITHER `classify`
    ///        has stopped being sign-sensitive OR the harness has collapsed the
    ///        two legs into one row. It NEVER means "the layer is fine", and
    ///        separating the two causes needs an instrument outside G9 (row
    ///        provenance) that this clause does NOT claim to supply.
    ///   (h3) The narrower aliasing — `leftMuscles` resolving to the same MODEL
    ///        muscles as `rightMuscles`, so the two rows are numerically equal
    ///        but still separately perturbed — does NOT hide here: G9(b2) would
    ///        still read 258. So G9(b2) does not close G9(a)'s hole either, and
    ///        it is registered without that claim.
    ///   (h4) It also reads 0, benignly, if `scored` itself reaches 0. `scored >
    ///        0` is therefore asserted FIRST, and a zero-scored run prints
    ///        VACUOUS-BY-CONSTRUCTION rather than counting as anything at all.
    ///   (h5) It is deliberately NOT self-fulfilling: the flip of `modeL` is
    ///        never asserted directly. Only the OBSERVABLE `modeR` vs `modeL` is
    ///        scored, so a classifier that stopped distinguishing lengthening
    ///        from shortening turns this RED instead of passing it.
    /// WHAT G9(b2) DOES NOT REPAIR, stated so the scope note is not overread: it
    /// bounds the SIGN class only. The MAGNITUDE class — a one-sided moment-arm
    /// error of realistic size — remains UNREACHED by any mode-agreement count on
    /// this fixture face, for the reason proved above, and rehoming it to a
    /// continuous-valued instrument is a separate preregistration that is
    /// deliberately NOT made here.
    ///
    /// ─── NAVIGATIONAL ADDENDUM, added 2026-08-22 (round 16 part C). It says
    /// only WHERE the registered text above now runs and alters NO registered
    /// statement, bar, number or verdict. ───
    /// G9(b2) executes in
    /// `testG9b2SignClassDiscriminationIsSensitiveToAOneSidedSignError`, the
    /// method immediately below this one. G9(b) — this method — is UNCHANGED:
    /// it still runs the multiplicative `1 + 1.114 %` control, still reads 0,
    /// and still pins that 0 as FAILED. SUPERSEDED-NOT-ERASED means both
    /// methods run, every run, and the red pin below stays red.
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

    /// ─── G9(b2) EXECUTION SITE, added 2026-08-22 (round 16 part C). The
    /// registered clause text on `testG9MirrorCheckIsSensitiveToAOneSidedError`
    /// is NOT edited; this comment adds only WHICH assertion carries which of
    /// its registered sentences. ───
    ///
    /// G9(b) stays exactly where it is, red-pinned, SUPERSEDED-NOT-ERASED. The
    /// BAR does not move — `RegisteredBar.g9RequiredDisagreements` is REUSED
    /// VERBATIM at 1 and re-asserted to be 1 here. The PERTURBATION CLASS does.
    ///
    /// THE ALGEBRA, RE-DERIVED AGAINST THE SHIPPED SOURCE ON 2026-08-22 rather
    /// than quoted (`MLM:` = `BioMotion/Muscle/MuscleLengthMode.swift`):
    ///
    ///     jitterMetres(R, σ)    = sqrt( Σⱼ (R[j]·σ[j])² )          MLM:266-274
    ///     stepDeadbandMetres(R) = max( k·g·jitterMetres(R,σ), F )  MLM:277-282
    ///     lengthRate(R, dq)     = − Σⱼ R[j]·dq[j]                  MLM:309-314
    ///     classify(v, D)        = lengthening if v > D;
    ///                             shortening if v < −D; else third MLM:319-324
    ///
    /// WHY G9(b)'s CONTROL CANNOT FAIL, in one line each: under `R → cR` with
    /// `c = 1 + 0.01114 > 0`, `jitterMetres` is HOMOGENEOUS OF DEGREE 1 so
    /// `s → c·s`, and `lengthRate` is LINEAR so `v → c·v`; in the `k·g·s` branch
    /// `D → c·D` and `c·v > c·D ⟺ v > D`, EXACTLY, so the verdict is invariant;
    /// in the `F` branch `D` is pinned while `|v|` grows, which can only move
    /// THIRD → DIRECTIONAL and never across the sign, because `sign(c·v) =
    /// sign(v)` for every `c > 0`.
    ///
    /// WHY THIS ONE CAN: under `R → −R`, `jitterMetres` sums SQUARES and
    /// IEEE-754 negation flips only a sign bit, so `s` and hence `D_L` are
    /// BIT-IDENTICAL; `lengthRate` is linear and odd and round-to-nearest-even
    /// is sign-symmetric, so `v_L → −v_L` and `|v_L|` is unchanged. The
    /// scored/excluded partition is therefore byte-identical to the unperturbed
    /// run's — which is why bar (ii) is assertable at all — while every
    /// DIRECTIONAL `modeL` moves to the OPPOSITE directional mode.
    ///
    /// WHAT RUNS HERE, one line per registered sentence:
    ///   * the control NEGATES one leg's moment-arm row in BOTH registered
    ///     forms — whole row, and the swept entry alone — and their claimed
    ///     observational equivalence is MEASURED across every field of
    ///     `G9Outcome`, not argued;
    ///   * bar (i)  `disagreements >= RegisteredBar.g9RequiredDisagreements`;
    ///   * bar (ii) `scored` and `excluded` equal the UNPERTURBED run's,
    ///     asserted against THAT RUN — recomputed here — never a literal;
    ///   * bar (iii) `disagreements == scored` and
    ///     `disagreementsOnUnperturbedExcludedSteps == 0`, plus the complement
    ///     `disagreementsOnUnperturbedScoredSteps == disagreements` so the
    ///     partition CLOSES rather than being half-read against an unstated
    ///     denominator;
    ///   * the PRE-REGISTERED PREDICTION `disagreements = 258 = scored,
    ///     excluded = 270`, asserted as the literals it was written as and kept
    ///     SEPARATE from bar (ii) so a red run names which of the two broke.
    ///
    /// THE ESCAPE HATCHES ARE NOT DECORATION. All five are printed on the
    /// MODE-VERDICT line on EVERY run, pass or fail, so no future round can
    /// quote this gate's reading as something it is not:
    ///   (h1) the 258 is ARITHMETICALLY FORCED by pins already in this file plus
    ///        the sign algebra above — this clause's outcome is determinable
    ///        from the repo WITHOUT running it. A MATCH is therefore NOT new
    ///        evidence of anything and may not be cited as any. The gate's
    ///        discriminating power lives ENTIRELY in its failure modes.
    ///        `discovery=NONE` is printed for exactly that reason.
    ///   (h2) it reads 0 despite a REAL defect precisely when the perturbation
    ///        lands UPSTREAM of the left/right split, so `modeR` sees the
    ///        flipped row too and both flip together — the left/right aliasing
    ///        G9(a) exists to catch and cannot itself see, because an aliased
    ///        pair agrees trivially. A 0 here means EITHER `classify` stopped
    ///        being sign-sensitive OR the harness collapsed the two legs onto
    ///        ONE row. It NEVER means "the layer is fine", and separating the
    ///        two causes needs row provenance — an instrument OUTSIDE G9 that
    ///        this clause does not claim to supply.
    ///   (h3) the narrower aliasing — `leftMuscles` resolving to the same MODEL
    ///        muscles, so the rows are numerically equal but still separately
    ///        perturbed — does NOT hide here, and is NOT closed by here either:
    ///        this gate would still read 258.
    ///   (h4) it also reads 0, benignly, if `scored` itself reaches 0. `scored >
    ///        0` is asserted FIRST and a zero-scored run prints
    ///        VACUOUS-BY-CONSTRUCTION and is RED, never a pass.
    ///   (h5) it is deliberately NOT self-fulfilling: the flip of `modeL` is
    ///        never asserted directly. Only the OBSERVABLE `modeR` vs `modeL`
    ///        disagreement is scored, so a classifier that stopped telling
    ///        lengthening from shortening turns this RED instead of passing it.
    ///
    /// SCOPE, unchanged from the registration and not to be overread: this
    /// bounds the SIGN class ONLY. The MAGNITUDE class — a one-sided moment-arm
    /// error of realistic size — stays UNREACHED by any mode-agreement count on
    /// this fixture face.
    func testG9b2SignClassDiscriminationIsSensitiveToAOneSidedSignError() throws {
        let ctx = try context()

        // The UNPERTURBED reference, RECOMPUTED here so bar (ii) can be asserted
        // against THAT RUN instead of against a literal.
        let baseline = try g9Outcome(ctx: ctx, perturbationRelative: 0)
        // The registered control, in BOTH of its registered forms.
        let flipped = try g9Outcome(ctx: ctx, perturbationRelative: 0, signFlip: .wholeRow)
        let sweptOnly = try g9Outcome(ctx: ctx, perturbationRelative: 0,
                                      signFlip: .sweptEntryOnly)

        // The receipts, printed BEFORE anything is adjudicated so a red run
        // still leaves a complete reading.
        let runs: [(String, G9Outcome)] = [("unperturbed_reference", baseline),
                                           ("whole_row_negation", flipped),
                                           ("swept_entry_only_negation", sweptOnly)]
        for (form, o) in runs {
            var line = "MODE-METRIC g9b2 form=\(form) pairs=\(o.pairs)"
            line += " scored=\(o.scored) excluded=\(o.excluded)"
            line += " disagree=\(o.disagreements)"
            line += " disagree_on_g9a_scored_steps=\(o.disagreementsOnUnperturbedScoredSteps)"
            line += " disagree_on_g9a_excluded_steps=\(o.disagreementsOnUnperturbedExcludedSteps)"
            line += " detail=\(o.detail.prefix(6).joined(separator: ","))"
            print(line)
        }

        // The PREMISE the 258 derivation rests on. PRINTED, not gated: its gate
        // is G9(a) itself, in `testG9BilateralMirrorCoherence`. Adding a second
        // copy of G9(a)'s bar here would be an unregistered bar.
        var premise = "MODE-METRIC g9b2 premise=g9a_unperturbed"
        premise += " disagreements=\(baseline.disagreements)"
        premise += " scored=\(baseline.scored) excluded=\(baseline.excluded)"
        premise += " note=the_258_prediction_is_FORCED_by_this_0_plus_the_sign_algebra__see_h1"
        print(premise)

        // (h4), asserted FIRST: a 0-disagreement reading on a 0-step population
        // is not a measurement, and must not be counted as one.
        guard flipped.scored > 0, sweptOnly.scored > 0 else {
            var vac = "MODE-VERDICT gate=G9(b2) outcome=VACUOUS-BY-CONSTRUCTION"
            vac += " population=\(flipped.scored)"
            vac += " swept_entry_population=\(sweptOnly.scored)"
            vac += " note=h4__0_==_0_scores_NOTHING__NOT_a_pass__NOT_evidence_the_mirror_check"
            vac += "_is_sign_sensitive"
            print(vac)
            XCTFail("G9(b2) NON-VACUITY (h4): the sign-flipped run scored no steps — RED, not "
                    + "skipped, and not a pass")
            return
        }

        // The adjudication, computed before it is printed.
        let barI = flipped.disagreements >= RegisteredBar.g9RequiredDisagreements
        let barII = flipped.scored == baseline.scored && flipped.excluded == baseline.excluded
        let barIIIa = flipped.disagreements == flipped.scored
        let barIIIb = flipped.disagreementsOnUnperturbedExcludedSteps == 0
        let barIIIc = flipped.disagreementsOnUnperturbedScoredSteps == flipped.disagreements
        let flippedOnScored = flipped.disagreementsOnUnperturbedScoredSteps
        let flippedOnExcluded = flipped.disagreementsOnUnperturbedExcludedSteps
        let sweptOnScored = sweptOnly.disagreementsOnUnperturbedScoredSteps
        let sweptOnExcluded = sweptOnly.disagreementsOnUnperturbedExcludedSteps
        var formsAgree = sweptOnly.scored == flipped.scored
        formsAgree = formsAgree && sweptOnly.excluded == flipped.excluded
        formsAgree = formsAgree && sweptOnly.disagreements == flipped.disagreements
        formsAgree = formsAgree && sweptOnScored == flippedOnScored
        formsAgree = formsAgree && sweptOnExcluded == flippedOnExcluded
        var predicted = flipped.scored == 258
        predicted = predicted && flipped.disagreements == 258
        predicted = predicted && flipped.excluded == 270

        var failing: [String] = []
        if !barI { failing.append("bar_i_disagreements_below_the_registered_minimum") }
        if !barII { failing.append("bar_ii_population_left_the_unperturbed_run") }
        if !barIIIa { failing.append("bar_iii_disagreements_!=_scored") }
        if !barIIIb { failing.append("bar_iii_disagreement_on_a_step_G9(a)_excludes") }
        if !barIIIc { failing.append("bar_iii_partition_does_not_close") }
        if !formsAgree { failing.append("the_two_registered_forms_are_not_observationally_equal") }
        if !predicted { failing.append("the_preregistered_258_258_270_prediction") }
        let outcome = failing.isEmpty ? "PASS_NON_VACUOUS" : "FAILED_AGAINST_REGISTERED_BAR"

        var verdict = "MODE-VERDICT gate=G9(b2) outcome=\(outcome) class=SIGN"
        verdict += " pairs=\(flipped.pairs) scored=\(flipped.scored)"
        verdict += " excluded=\(flipped.excluded) disagree=\(flipped.disagreements)"
        verdict += " unperturbed_scored=\(baseline.scored)"
        verdict += " unperturbed_excluded=\(baseline.excluded)"
        verdict += " swept_entry_only=[scored=\(sweptOnly.scored)"
        verdict += " excluded=\(sweptOnly.excluded) disagree=\(sweptOnly.disagreements)]"
        verdict += " partition=[on_g9a_scored=\(flipped.disagreementsOnUnperturbedScoredSteps)"
        verdict += " on_g9a_excluded=\(flipped.disagreementsOnUnperturbedExcludedSteps)"
        verdict += " denominator=\(flipped.disagreements)"
        verdict += " vacuous=\(flipped.disagreements == 0)]"
        verdict += " failing=\(failing)"
        verdict += " registered_bar=[(i) disagreements >= "
        verdict += "\(RegisteredBar.g9RequiredDisagreements), the G9(b) constant REUSED VERBATIM;"
        verdict += " (ii) scored and excluded EQUAL the unperturbed run's, asserted against THAT"
        verdict += " RUN; (iii) disagreements == scored, 0 on steps G9(a) excludes, partition"
        verdict += " closes]"
        verdict += " prediction=[disagreements=258=scored, excluded=270]"
        verdict += " supersedes=[G9(b), the multiplicative 1.114 % control — FAILED,"
        verdict += " SUPERSEDED-NOT-ERASED, still pinned red in"
        verdict += " testG9MirrorCheckIsSensitiveToAOneSidedError]"
        verdict += " determinable_without_running=true discovery=NONE"
        verdict += " h1=[the 258 is ARITHMETICALLY FORCED by the pins already in this file plus"
        verdict += " the sign algebra; a MATCHING reading is NOT new evidence of anything and"
        verdict += " may never be cited as any; this clause's discriminating power lives"
        verdict += " ENTIRELY in its FAILURE modes]"
        verdict += " h2=[a 0 reading NEVER means the layer is fine — it means EITHER classify"
        verdict += " stopped being sign-sensitive OR the harness collapsed the two legs onto ONE"
        verdict += " row, i.e. the left/right aliasing G9(a) exists to catch and cannot itself"
        verdict += " see; separating those two causes needs row provenance, an instrument"
        verdict += " OUTSIDE G9 that this clause does not supply]"
        verdict += " h3=[does NOT close G9(a)'s aliasing hole either: numerically equal but"
        verdict += " separately perturbed rows still read 258]"
        verdict += " h4=[scored > 0 asserted FIRST; a 0-scored run prints"
        verdict += " VACUOUS-BY-CONSTRUCTION and is RED, never a pass]"
        verdict += " h5=[not self-fulfilling: the modeL flip is NEVER asserted directly, only"
        verdict += " the OBSERVABLE modeR vs modeL disagreement is scored]"
        verdict += " claims=[the mirror check is sensitive to the SIGN class]"
        verdict += " claims_not=[the MAGNITUDE class, which stays UNREACHED on this fixture face]"
        verdict += " first_measurement=true provenance=FullBody.osim+opensim_moment_arms.txt"
        print(verdict)

        if !failing.isEmpty {
            recordFailedGate("G9(b2)",
                             measured: "sign-flipped scored=\(flipped.scored) "
                                       + "excluded=\(flipped.excluded) "
                                       + "disagree=\(flipped.disagreements) against unperturbed "
                                       + "scored=\(baseline.scored) "
                                       + "excluded=\(baseline.excluded); swept-entry form "
                                       + "scored=\(sweptOnly.scored) "
                                       + "disagree=\(sweptOnly.disagreements); failing=\(failing)",
                             bar: "(i) >= \(RegisteredBar.g9RequiredDisagreements) disagreement; "
                                  + "(ii) scored and excluded equal the unperturbed run's; "
                                  + "(iii) disagreements == scored and 0 on steps G9(a) excludes",
                             why: "the SIGN class is the one the frozen registration assigns to "
                                  + "G9 and the one the 173-pose cylinder-wrap receipt actually "
                                  + "produced; a reading other than disagreements == scored means "
                                  + "EITHER classify stopped being sign-sensitive OR the harness "
                                  + "collapsed the two legs onto one row (h2) — it never means "
                                  + "the layer is fine")
        }

        // ─── NON-VACUITY (h4) and the population, asserted before any bar. ───
        XCTAssertGreaterThan(flipped.scored, 0,
            "G9(b2) NON-VACUITY: the sign-flipped run scored no steps — RED, not skipped")
        XCTAssertGreaterThan(sweptOnly.scored, 0,
            "G9(b2) NON-VACUITY: the swept-entry form scored no steps — RED, not skipped")
        XCTAssertEqual(flipped.pairs, 16, "the 16 displayed bilateral pairs moved")

        // ─── BAR (i): REUSED VERBATIM from G9(b). The value is re-asserted here
        // so a silent edit to the constant turns BOTH gates red, not one. ───
        XCTAssertEqual(RegisteredBar.g9RequiredDisagreements, 1,
            "G9(b2) reuses G9(b)'s bar VERBATIM: the perturbed re-run must disagree at least once")
        XCTAssertGreaterThanOrEqual(flipped.disagreements,
                                    RegisteredBar.g9RequiredDisagreements,
            "G9(b2)(i): the SIGN-class control produced no disagreement. Per (h2) this NEVER "
            + "means the layer is fine — it means EITHER classify stopped being sign-sensitive "
            + "OR the harness collapsed the two legs onto one moment-arm row")

        // ─── BAR (ii): against THAT RUN, never against a literal, so the
        // population is comparable with G9(a)'s BY CONSTRUCTION. ───
        XCTAssertEqual(flipped.scored, baseline.scored,
            "G9(b2)(ii): the sign flip moved the SCORED population off the unperturbed run's — "
            + "it cannot, because jitterMetres sums squares and IEEE-754 negation only flips a "
            + "sign bit, so every deadband is bit-identical")
        XCTAssertEqual(flipped.excluded, baseline.excluded,
            "G9(b2)(ii): the sign flip moved the EXCLUDED population off the unperturbed run's")

        // ─── BAR (iii): coherence, with the partition CLOSED on both sides so
        // the 0 half is read against a stated denominator. ───
        XCTAssertEqual(flipped.disagreements, flipped.scored,
            "G9(b2)(iii): under a sign flip EVERY scored step must disagree, because OR-scoring "
            + "plus G9(a)'s pinned 0 forces both legs to the SAME directional mode first")
        XCTAssertEqual(flipped.disagreementsOnUnperturbedExcludedSteps, 0,
            "G9(b2)(iii): a disagreement landed on a step G9(a) EXCLUDES, so the partition is "
            + "not byte-identical to the unperturbed run's — read against a denominator of "
            + "\(flipped.disagreements) disagreements, NOT against an empty population")
        XCTAssertEqual(flipped.disagreementsOnUnperturbedScoredSteps, flipped.disagreements,
            "G9(b2)(iii): the partition does not close — this is the NON-VACUOUS half of the "
            + "same split and it must carry all \(flipped.disagreements) of them")

        // ─── BOTH REGISTERED FORMS, equivalence MEASURED rather than argued. ───
        XCTAssertEqual(sweptOnly.scored, flipped.scored,
            "G9(b2): negating the swept entry alone is registered as observationally IDENTICAL "
            + "to negating the whole row on this single-DOF drive; the scored populations differ")
        XCTAssertEqual(sweptOnly.excluded, flipped.excluded,
            "G9(b2): the two registered forms disagree on the EXCLUDED population")
        XCTAssertEqual(sweptOnly.disagreements, flipped.disagreements,
            "G9(b2): the two registered forms disagree on the DISAGREEMENT count — dq has "
            + "exactly one non-zero entry, so they cannot")
        XCTAssertEqual(sweptOnly.disagreementsOnUnperturbedScoredSteps,
                       flipped.disagreementsOnUnperturbedScoredSteps,
            "G9(b2): the two registered forms split their disagreements differently")
        XCTAssertEqual(sweptOnly.disagreementsOnUnperturbedExcludedSteps,
                       flipped.disagreementsOnUnperturbedExcludedSteps,
            "G9(b2): the two registered forms split their disagreements differently")

        // ─── THE PRE-REGISTERED PREDICTION, written before the run, asserted as
        // the literals it was written as. SEPARATE from bar (ii) so a red run
        // names which of the two broke. Per (h1) a MATCH here is not evidence. ───
        XCTAssertEqual(flipped.scored, 258,
            "G9(b2) PREDICTION: 33 steps x 16 pairs = 528 = 258 scored + 270 excluded, and the "
            + "sign flip leaves that partition byte-identical")
        XCTAssertEqual(flipped.disagreements, 258,
            "G9(b2) PREDICTION: disagreements = 258 = scored")
        XCTAssertEqual(flipped.excluded, 270,
            "G9(b2) PREDICTION: excluded = 270")
    }

    /// The SIGN-CLASS perturbation G9(b2) registers, in the TWO observationally
    /// equivalent forms the registration names. `.unperturbed` compiles to
    /// `break`, so G9(a)'s and G9(b)'s existing call sites are unchanged.
    private enum G9SignFlip {
        /// No sign perturbation. The default; what G9(a) and G9(b) both use.
        case unperturbed
        /// `R_L → −R_L`. `jitterMetres` sums squares, so `D_L` is BIT-IDENTICAL.
        case wholeRow
        /// Negate ONLY the column of the coordinate this sweep drives. `dq` has
        /// exactly one non-zero entry, so `lengthRate` sees the same flip.
        case sweptEntryOnly
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
                           additiveRelative: Double = 0,
                           signFlip: G9SignFlip = .unperturbed) throws -> G9Outcome {
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
            // G9(b2)'s swept-entry form needs the column of the ONE coordinate
            // this sweep drives. Hoisted out of the inner loop; `nil` here is a
            // harness defect, not a model finding, and is reported as such.
            let sweptLeftIndex = leftCoords.firstIndex(of: sweep.left)
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
                    // G9(b2)'s SIGN class. `.unperturbed` is `break`, so G9(a)'s
                    // and G9(b)'s call sites are byte-identical to what they were.
                    switch signFlip {
                    case .unperturbed:
                        break
                    case .wholeRow:
                        rowL = rowL.map { -$0 }
                    case .sweptEntryOnly:
                        guard let swept = sweptLeftIndex else {
                            XCTFail("G9(b2): \(sweep.left) is not in the left span union, so the "
                                    + "swept-entry form has nothing to negate — a HARNESS defect, "
                                    + "not a reading about the layer")
                            return outcome
                        }
                        rowL[swept] = -rowL[swept]
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
            // TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
            // amendment): 122 -> 120. The 5-marker `GaitClipFixture` lineage
            // carried 122 frames; the video-driven 20-marker lineage samples
            // `.nativeWindow(4.0)` at 30 fps, i.e. `min(120, available)` = 120,
            // and retained ALL of them on both clips (branch A, `excluded=[]`).
            // Provenance: the person box is macOS-Vision INTERIM (see the
            // fixture header's `bbox_source`), so this is an interim substrate.
            XCTAssertEqual(fixture.frames.count, 240)
            XCTAssertEqual(fixture.markerNames.count, 20,
                           "the 20-marker MHR drive is what this lineage exists for")
            XCTAssertEqual(fixture.distinctIntervals.count, 1,
                           "the clip is not uniformly sampled")
        }
    }

    // MARK: - G2: temporal stability and non-degeneracy

    /// G2(a)+(e). Flicker `<= 1.0 %`, grey-transition `<= 2.0 %`, scored
    /// SEPARATELY per clip.
    ///
    /// ⚠️ **This method WAS GREEN VACUOUSLY on the 5-marker fixtures, and that
    /// was never a satisfied stability bar.** Both denominators were 0 because no
    /// capsule was admitted, so `flickerRate` and `greyTransitionRate` returned
    /// the `0` of an empty ratio and cleared their bars without measuring
    /// anything. That vacuity was asserted and recorded — as vacuity — in
    /// `testG2NonDegeneracyOnThePinnedClips`.
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment; STATUS next-step 41 (a) discipline) ───
    /// **Both denominators are now NON-EMPTY and both clauses FAIL on real
    /// measurements.** The 20-marker video-driven fixtures admit 14 capsules on
    /// `video_012` and 24 on `video_015`, so the empty-ratio escape is gone and
    /// what was vacuous green is now a measured red. BOTH BARS ARE UNCHANGED and
    /// still read out of `RegisteredBar`; the assertions below record the
    /// measured outcome instead of demanding the bar, and the `MODE-VERDICT`
    /// line states the failure. Receipt:
    /// `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`, `grep 'MODE-METRIC g2a'`.
    ///
    /// PROVENANCE: the person box in these fixtures is **macOS Vision, INTERIM**
    /// (`bbox_source macos_vision INTERIM` in the fixture header). Nothing
    /// recorded here is quotable as device-grade.
    func testG2TemporalStabilityOnThePinnedClips() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g2FlickerRate, 0.01, "G2(a)'s registered bar")
        XCTAssertEqual(RegisteredBar.g2GreyTransitionRate, 0.02, "G2(e)'s registered bar")

        // The MEASURED outcome, pinned to the receipt. Bidirectional: a number
        // that drifts AND a number that improves past its pin both turn this
        // red, because either means the mask, the fixtures, the classifier or
        // the model moved and the verdict must be re-adjudicated.
        let pinned: [String: (flickerCentres: Int, flickerDenominator: Int,
                              greyTransitions: Int, greyDenominator: Int)] = [
            "video_012": (71, 2816, 148, 3234),
            "video_015": (152, 4907, 292, 5544),
        ]
        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g2a clip=\(clip) flicker_centres=\(t.flickerCentres) "
                  + "flicker_denominator=\(t.flickerDenominator) "
                  + String(format: "flicker_rate=%.6f", t.flickerRate)
                  + " grey_transitions=\(t.greyTransitions) grey_denominator=\(t.greyDenominator) "
                  + String(format: "grey_rate=%.6f", t.greyTransitionRate))

            // ─── THE NEXT-STEP-51 DIAGNOSTIC. NOT A GATE, AND IT MUST NOT
            // BECOME ONE IN THIS ROUND. ───
            // STATUS next-step 51 forbids proposing a fix for the G2(a)/G2(e)
            // failures above before MEASURING which of three causes they are.
            // This prints that measurement and asserts NOTHING about it: any bar
            // written now would be a bar chosen after already seeing the worst
            // flicker rate 65/1347 = 4.8255 % against a 1 % bar and the worst
            // grey rate 150/2664 = 5.6306 % against a 2 % bar (both re-derived
            // by hand 2026-08-21 from the pins on :1892-1893), which is the same
            // move as editing one. How the census is read, stated here as well
            // as on `MuscleModeFlipEvent` so a reader of the log finds it in the
            // source next to the numbers it explains:
            //
            //   * breaker `|v| / D` clustered JUST ABOVE 1.0 at the flips ⇒ the
            //     capsules are crossing their own deadband edge, i.e. a
            //     DEADBAND / POSE-NOISE problem, and the pose-noise control line
            //     (`resid_z_max_median_at_flip` against `..._all`) says whether
            //     the flip frames are noisier than the clip in general;
            //   * breaker `|v| / D` LARGE — and large on BOTH sides of an
            //     L↔S reversal — ⇒ the length rate genuinely reversed, i.e. a
            //     CLASSIFIER / KINEMATICS problem, which no deadband or dwell
            //     layer would repair;
            //   * `rule3=…>1` ⇒ NEITHER of those: that frame was forced
            //     `.indeterminate` by an unresolved wrap switch. Counted apart
            //     so it cannot be silently attributed to the other two.
            //
            // It runs off the SAME cached traversal `t` above came from, so it
            // enumerates exactly the `flicker_centres` + `grey_transitions`
            // events this clip's own pins record — 65 + 67 on `video_012`,
            // 71 + 150 on `video_015` — and not a re-run of anything.
            print(Self.flipCensusReport(clip: clip))

            let pin = try XCTUnwrap(pinned[clip])
            XCTAssertEqual(t.flickerCentres, pin.flickerCentres, "G2(a) pin on \(clip)")
            XCTAssertEqual(t.flickerDenominator, pin.flickerDenominator, "G2(a) pin on \(clip)")
            XCTAssertEqual(t.greyTransitions, pin.greyTransitions, "G2(e) pin on \(clip)")
            XCTAssertEqual(t.greyDenominator, pin.greyDenominator, "G2(e) pin on \(clip)")

            // NON-VACUITY, asserted BEFORE the verdict: an empty denominator
            // would make either rate a 0 that clears its bar without measuring.
            XCTAssertGreaterThan(t.flickerDenominator, 0,
                                 "G2(a) on \(clip) must not be scored on an empty denominator")
            XCTAssertGreaterThan(t.greyDenominator, 0,
                                 "G2(e) on \(clip) must not be scored on an empty denominator")

            // The VERDICT.
            XCTAssertGreaterThan(t.flickerRate, RegisteredBar.g2FlickerRate,
                                 "G2(a) is recorded as FAILED on \(clip)")
            XCTAssertGreaterThan(t.greyTransitionRate, RegisteredBar.g2GreyTransitionRate,
                                 "G2(e) is recorded as FAILED on \(clip)")
            recordFailedGate("G2(a)+G2(e)", clip: clip,
                             measured: String(format: "flicker=%d/%d=%.6f grey=%d/%d=%.6f "
                                              + "(NON-VACUOUS: both denominators are populated)",
                                              t.flickerCentres, t.flickerDenominator, t.flickerRate,
                                              t.greyTransitions, t.greyDenominator,
                                              t.greyTransitionRate),
                             bar: "flicker <= \(RegisteredBar.g2FlickerRate), "
                                  + "grey <= \(RegisteredBar.g2GreyTransitionRate)",
                             why: "the 20-marker drive finally produces a capsule population, and "
                                  + "on it the per-frame mode is not temporally stable at the "
                                  + "registered rates; bbox_source=macos_vision INTERIM")
        }
    }

    /// G2(b)(c)(d)(f). Non-degeneracy and the power floor. Flicker alone is
    /// passed PERFECTLY by an all-third-state or all-INDETERMINATE output and is
    /// invariant under sign inversion, so without these this gate rewards a layer
    /// that says nothing.
    ///
    /// ─── CONVERTED 2026-08-14 (owner-authorised, STATUS next-step 41 (a)) ───
    /// All four bars are UNCHANGED and still asserted through `RegisteredBar`.
    /// The 2026-08-13 conversion RECORDED an empty population: on the 5-marker
    /// fixtures every figure was EXACTLY ZERO on both clips — 0 admitted
    /// capsules, 0 defined samples, 0 total samples, no per-capsule entry — and
    /// G2(a)/(e) were vacuous by empty denominator. Those pins were declared
    /// bidirectional precisely so that "the day a capsule IS admitted these pins
    /// go red".
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ─── **That day arrived, and the pins went red as designed.**
    /// The 20-marker video-driven fixtures admit **14** capsules on `video_012`
    /// and **24** on `video_015`. Three of the four clauses now clear their
    /// registered bars on a NON-EMPTY population, and one does not:
    ///
    /// * **G2(b) PASSES** — directional `0.423077` (012) / `0.523587` (015) vs `>= 0.40`
    /// * **G2(d) PASSES** — defined fraction `0.928571` / `0.954241` vs `>= 0.90`
    /// * **G2(f) PASSES** — defined samples `1456` / `2565` vs `>= 300`
    /// * **G2(c) FAILS on `video_012`** — the weakest capsule is `gaslat_r` /
    ///   `gasmed_r` at `0.008929`, an order of magnitude under the `>= 0.10`
    ///   bar; on `video_015` the weakest is `glmed1_r` at `0.200000` and the
    ///   clause passes. A clause met on one clip and missed on the other is a
    ///   FAIL for the round: the registered wire condition is BOTH clips.
    ///
    /// Receipt: `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g2bcdf'`. Bars untouched; only the measured-outcome
    /// pins moved.
    ///
    /// PROVENANCE: **macOS Vision, INTERIM** (`bbox_source macos_vision INTERIM`).
    /// The layer is still NOT SHIPPED and no UI is wired.
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

            // The MEASURED outcome, pinned to the receipt. TRANSITIONED
            // 2026-08-14 from the all-zero 5-marker pins.
            let pin = try XCTUnwrap(Self.g2Pins[clip])
            XCTAssertEqual(t.admittedCapsules.count, pin.admitted, "G2 pin on \(clip)")
            XCTAssertEqual(t.definedSamples, pin.defined, "G2(f) pin on \(clip)")
            XCTAssertEqual(t.totalSamples, pin.total, "G2 pin on \(clip)")
            XCTAssertEqual(t.directionalFraction, pin.directional, accuracy: 1e-9,
                           "G2(b) pin on \(clip)")
            XCTAssertEqual(t.definedFraction, pin.definedFraction, accuracy: 1e-9,
                           "G2(d) pin on \(clip)")
            XCTAssertEqual(t.perCapsuleDirectionalFraction.count, pin.admitted,
                           "G2(c) pin on \(clip): one entry per admitted capsule")
            XCTAssertEqual(t.minimumCapsuleDirectionalFraction, pin.minimumCapsule,
                           accuracy: 1e-9, "G2(c) pin on \(clip)")
            XCTAssertEqual(perCapsule.first(where: { $0.hasPrefix(pin.weakestCapsule + "=") }),
                           String(format: "%@=%.4f", pin.weakestCapsule, pin.minimumCapsule),
                           "G2(c) pin on \(clip): the weakest capsule moved")

            // NON-VACUITY, asserted BEFORE any bar is read. A bar met on an
            // empty population is a VACUOUS pin, never a live bar; that rule is
            // what the 5-marker round's zeros existed to enforce, and it is what
            // licenses reading the PASSES below as real.
            XCTAssertGreaterThan(t.admittedCapsules.count, 0,
                                 "G2 on \(clip): no capsule is admitted, so nothing below is a "
                                 + "measurement")
            XCTAssertGreaterThan(t.totalSamples, 0, "G2 on \(clip): no capsule sample exists")

            // The VERDICT, clause by clause. THREE PASS on a populated set.
            XCTAssertGreaterThanOrEqual(t.definedSamples, RegisteredBar.g2DefinedSamples,
                                        "G2(f) is recorded as PASSED on \(clip)")
            XCTAssertGreaterThanOrEqual(t.directionalFraction, RegisteredBar.g2DirectionalFraction,
                                        "G2(b) is recorded as PASSED on \(clip)")
            XCTAssertGreaterThanOrEqual(t.definedFraction, RegisteredBar.g2DefinedFraction,
                                        "G2(d) is recorded as PASSED on \(clip)")
            print("MODE-VERDICT gate=G2(b)+G2(d)+G2(f) clip=\(clip) outcome=PASS_NON_VACUOUS "
                  + "admitted=\(t.admittedCapsules.count) defined=\(t.definedSamples) "
                  + String(format: "directional=%.6f defined_fraction=%.6f",
                           t.directionalFraction, t.definedFraction)
                  + " bbox_source=macos_vision INTERIM "
                  + "note=first non-empty capsule population in this battery's history; "
                  + "macOS Vision provenance is NOT iOS Vision provenance")

            // G2(c) is the one clause that does not clear, and it clears on ONE
            // clip only — which is a FAIL for the round, because the registered
            // wire condition is BOTH clips.
            if pin.minimumCapsule >= RegisteredBar.g2CapsuleDirectionalFraction {
                print("MODE-VERDICT gate=G2(c) clip=\(clip) outcome=PASS_NON_VACUOUS "
                      + String(format: "minimum=%.6f capsule=%@", t.minimumCapsuleDirectionalFraction,
                               pin.weakestCapsule)
                      + " bar=>= \(RegisteredBar.g2CapsuleDirectionalFraction) "
                      + "note=PASSES on this clip only; the round's wire condition is BOTH clips")
                XCTAssertGreaterThanOrEqual(t.minimumCapsuleDirectionalFraction,
                                            RegisteredBar.g2CapsuleDirectionalFraction,
                                            "G2(c) is recorded as PASSED on \(clip)")
            } else {
                XCTAssertLessThan(t.minimumCapsuleDirectionalFraction,
                                  RegisteredBar.g2CapsuleDirectionalFraction,
                                  "G2(c) is recorded as FAILED on \(clip)")
                recordFailedGate("G2(c)", clip: clip,
                                 measured: String(format: "minimum=%.6f on %@ over %d admitted "
                                                  + "capsules (NON-VACUOUS)",
                                                  t.minimumCapsuleDirectionalFraction,
                                                  pin.weakestCapsule, t.admittedCapsules.count),
                                 bar: "every capsule >= \(RegisteredBar.g2CapsuleDirectionalFraction)",
                                 why: "the two gastrocnemius capsules are admitted but almost never "
                                      + "directional on this clip; bbox_source=macos_vision INTERIM")
            }
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
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ─── The fourteenth-round text above is kept as history; the
    /// numbers it recites (0 joint frames, ceiling 0, the BoBjif receipt) are
    /// the FIVE-marker fixtures' and no longer describe the pins below. On the
    /// 20-marker video-driven fixtures the population is real for the first
    /// time: agreement `1.000000` on BOTH clips, `rule3_excluded 0`,
    /// signature changes `152` / `218` — but joint-clearing frames land at
    /// **316** (ceiling 1998, 18 admitted x 111 pairs) on `video_012` and
    /// **398** (ceiling 3552, 32 x 111) on `video_015`, both UNDER the held
    /// 500-frame power floor. VERDICT: still FAILED (under-power), now for a
    /// measured reason, not a vacuous one; the perfect agreement on 714 real
    /// frames is evidence the two witnesses do not disagree, not yet evidence
    /// at registered power. Receipt: `/tmp/biomotion-tests.4Eb2JD/subset/
    /// xcodebuild.log`, `grep 'MODE-METRIC g7a'`. Bars untouched.
    /// PROVENANCE: macOS Vision, INTERIM. The layer is still NOT SHIPPED.
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

            // The MEASURED outcome, pinned to the receipt. TRANSITIONED
            // 2026-08-14 (fifteenth round) from the all-zero 5-marker
            // population. `warmedCount` moves 114 -> 112 because the fixture is
            // 240 samples rather than 242 and `warmedFrameCount = n - (taps-1)`.
            let pin = try XCTUnwrap(Self.g7Pins[clip])
            XCTAssertEqual(t.warmedCount, 232, "G7 pin on \(clip): the warmed-frame count moved")
            XCTAssertEqual(t.admittedMuscles.count, pin.admittedMuscles, "G7 pin on \(clip)")
            XCTAssertEqual(t.witnessJointFrames, pin.jointFrames, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.witnessAgreements, pin.agreements, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.witnessAgreement, 1.0, accuracy: 1e-12, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.rule3Excluded, 0, "G7(a) pin on \(clip)")
            XCTAssertEqual(t.signatureChanges, pin.signatureChanges, "G7(a) pin on \(clip)")
            XCTAssertEqual(ceiling, pin.ceiling, "G7(a) pin on \(clip)")

            // DECISION COHERENCE, half one: the admitted set is NON-EMPTY on
            // both clips now, and that is what makes the agreement figure a
            // measurement rather than an empty ratio.
            XCTAssertEqual(t.admittedCapsules.count, pin.admittedCapsules,
                           "coherence: the admitted capsule count must match G2's own pin")
            XCTAssertGreaterThan(t.witnessJointFrames, 0,
                                 "G7(a) on \(clip): an empty joint-clearing set would make the "
                                 + "agreement a 0/0 that says nothing")

            // ─── THE VERDICT, FLIPPED 2026-08-21 (sixteenth round) ───
            // For two rounds this clause asserted its OWN FAILURE: agreement was
            // perfect (1.000000) but 316/398 jointly-clearing frames sat under
            // the registered 500 floor, and a clause met on a population too
            // small to power it is not a pass. At the 8.0 s window the SAME
            // untouched floor is cleared on both clips — 715 (ceiling 4158) and
            // 766 (ceiling 7392), agreement still exactly 1.000000 — so the
            // assertion becomes the REAL GATE it was always a placeholder for.
            //
            // THE BAR DID NOT MOVE. `RegisteredBar.g7WitnessJointFrames` is
            // still 500 and `g7WitnessAgreement` still 0.99, both asserted two
            // lines below where they are read. What moved is the DRIVE: a
            // longer analysis window, pre-registered on the drift screen's
            // stride requirement before this measurement ran. The superseded
            // under-power text is kept above in this method's doc comment,
            // SUPERSEDED-NOT-ERASED, as the receipt of what 112 warmed frames
            // could and could not support.
            XCTAssertGreaterThanOrEqual(t.witnessAgreement, RegisteredBar.g7WitnessAgreement,
                "G7(a) agreement clause on \(clip)")
            XCTAssertGreaterThanOrEqual(t.witnessJointFrames, RegisteredBar.g7WitnessJointFrames,
                "G7(a) power clause on \(clip): the registered floor is the gate now, "
                + "not a recorded failure")
            print("MODE-VERDICT gate=G7(a) clip=\(clip) outcome=PASS_NON_VACUOUS"
                  + " joint_clearing=\(t.witnessJointFrames)"
                  + " agree=\(t.witnessAgreements) agreement=1.000000"
                  + " ceiling=\(ceiling) warmed=\(t.warmedCount)"
                  + " registered_bar=[>= \(RegisteredBar.g7WitnessJointFrames) jointly-clearing"
                  + " muscle-frames at >= \(RegisteredBar.g7WitnessAgreement) agreement]"
                  + " superseded=[4.0s_window_316_and_398_UNDER_POWER]"
                  + " provenance=macos_vision_INTERIM")
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
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ─── The vacuity narrative above is HISTORY, kept per the
    /// append-beside rule; it described the 5-marker fixtures. On the 20-marker
    /// video-driven fixtures the early return is NOT taken (`admitted 18` on
    /// `video_012`, `32` on `video_015`), the probe genuinely runs, and BOTH
    /// clauses clear their unchanged bars on a non-empty population:
    /// `min_length_range 1.639545072e-02 m` / `2.249332070e-02 m` (bar
    /// `> 1e-6`), `reimpose_max_delta 0.000e+00` (bar `<= 1e-9`) — the same
    /// digits as the vacuous round, opposite meaning: this time the block
    /// executed and wrote them. The body below asserts both clauses live and
    /// prints `outcome=PASS_NON_VACUOUS` behind an explicit
    /// `admittedMuscles.count > 0` population assertion.
    /// VERDICT: G7(b) PASSES non-vacuously on both clips. Receipt:
    /// `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g7b'`. Bars untouched.
    /// PROVENANCE: macOS Vision, INTERIM. The layer is still NOT SHIPPED.
    func testG7StalePoseSentinel() throws {
        // The registered bars, asserted where the gate reads them.
        XCTAssertEqual(RegisteredBar.g7MinimumLengthRangeMetres, 1.0e-6, "G7(b)'s registered range")
        XCTAssertEqual(RegisteredBar.g7ReimposedPoseDeltaMetres, 1.0e-9, "G7(b)'s registered delta")

        for clip in Self.scoredClips {
            let t = try Self.traversal(clip: clip, context: context())
            print("MODE-METRIC g7b clip=\(clip) admitted_muscles=\(t.admittedMuscles.count) "
                  + String(format: "min_length_range=%.9e reimpose_max_delta=%.3e",
                           t.minimumLengthRange, t.reimposedPoseMaxDelta))

            // The MEASURED outcome, pinned to the receipt. TRANSITIONED
            // 2026-08-14 (fifteenth round).
            let pin = try XCTUnwrap(Self.g7Pins[clip])
            XCTAssertEqual(t.admittedMuscles.count, pin.admittedMuscles, "G7(b) pin on \(clip)")
            XCTAssertEqual(t.minimumLengthRange, pin.minimumLengthRange, accuracy: 1e-12,
                           "G7(b) pin on \(clip)")
            XCTAssertEqual(t.reimposedPoseMaxDelta, 0.0, accuracy: 0, "G7(b) pin on \(clip)")

            // NON-VACUITY — and this is exactly the clause that WAS vacuous.
            // `buildTraversal` returns BEFORE the re-impose probe when the
            // admitted set is empty, so on the 5-marker fixtures the 0.000e+00
            // was an initial value. With \(pin.admittedMuscles) admitted muscles
            // the early return is not taken and the probe genuinely runs, so
            // this 0 is now a MEASUREMENT: re-imposing a stored mid-clip pose
            // reproduces every L_MT bit for bit.
            XCTAssertGreaterThan(t.admittedMuscles.count, 0,
                                 "G7(b) on \(clip): without an admitted muscle the sentinel's "
                                 + "0.000e+00 would be an initial value, not a measurement")

            // The VERDICT: BOTH clauses are met, on a real population.
            XCTAssertGreaterThan(t.minimumLengthRange, RegisteredBar.g7MinimumLengthRangeMetres,
                "G7(b) range clause on \(clip): every admitted trace must move more than 1e-6 m")
            XCTAssertLessThanOrEqual(t.reimposedPoseMaxDelta,
                                     RegisteredBar.g7ReimposedPoseDeltaMetres,
                "G7(b) re-impose clause on \(clip)")
            print("MODE-VERDICT gate=G7(b) clip=\(clip) outcome=PASS_NON_VACUOUS "
                  + "admitted_muscles=\(t.admittedMuscles.count) "
                  + String(format: "min_length_range=%.9e reimpose_max_delta=%.3e",
                           t.minimumLengthRange, t.reimposedPoseMaxDelta)
                  + " bar=range > \(RegisteredBar.g7MinimumLengthRangeMetres) m and re-impose <= "
                  + "\(RegisteredBar.g7ReimposedPoseDeltaMetres) m "
                  + "bbox_source=macos_vision INTERIM "
                  + "note=the probe RAN this time; on the 5-marker fixtures the same 0.000e+00 was "
                  + "an initial value because buildTraversal returned before it")
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
    ///
    /// ─── TRANSITIONED 2026-08-14 (fifteenth round, person-box sidecar
    /// amendment) ─── The vacuity narrative above is HISTORY, kept per the
    /// append-beside rule; it described the 5-marker fixtures. On the 20-marker
    /// video-driven fixtures the screen finally screens something — and BINDS:
    /// `video_012` scores 18 muscles, worst `0.297053` (`gasmed_r`) clears the
    /// 30 % primary, but **7 of 18** exceed the 20 % secondary against an
    /// allowance of **1** (`>= 90 %` of 18 must sit under 20 %, so at most
    /// `floor(0.10 x 18) = 1` may exceed), and the clause FAILS;
    /// `video_015` scores 32, worst `0.152928`
    /// (`glmax3_l`), 0 over 20 %, and PASSES. One clip passing is not the round
    /// passing: the registered condition is BOTH. The body pins both outcomes
    /// bidirectionally behind an explicit `trendExcursions.count > 0`
    /// population assertion. VERDICT: G8(a) FAILED on `video_012` (secondary
    /// clause), measured, non-vacuous. Receipt:
    /// `/tmp/biomotion-tests.4Eb2JD/subset/xcodebuild.log`,
    /// `grep 'MODE-METRIC g8a'`. Bars untouched.
    /// PROVENANCE: macOS Vision, INTERIM. The layer is still NOT SHIPPED.
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

            // The MEASURED outcome, pinned to the receipt. TRANSITIONED
            // 2026-08-14 (fifteenth round) from the empty 5-marker population.
            let pin = try XCTUnwrap(Self.g8Pins[clip])
            XCTAssertEqual(t.trendExcursions.count, pin.muscles, "G8(a) pin on \(clip)")
            XCTAssertEqual(sorted.first?.value ?? 0, pin.worst, accuracy: 1e-12,
                           "G8(a) pin on \(clip)")
            XCTAssertEqual(sorted.first?.key, pin.worstMuscle,
                           "G8(a) pin on \(clip): the worst muscle moved")
            XCTAssertEqual(over, pin.over20, "G8(a) pin on \(clip)")

            // NON-VACUITY: the 30 %/20 % clauses are only readable against a
            // real trace population.
            XCTAssertGreaterThan(t.trendExcursions.count, 0,
                                 "G8(a) on \(clip): an empty screen's `worst` is the collection's "
                                 + "own 0, not an excursion")

            // ─── THE VERDICT, RE-ADJUDICATED 2026-08-21 (sixteenth round) ───
            // SUPERSEDED-NOT-ERASED: "The PRIMARY clause holds on both clips;
            // the SECONDARY allowance is blown on video_012 alone." That was the
            // 4 s window's outcome (worst 0.2971 on video_012, 0.1529 on
            // video_015). At the 8 s window video_012's worst rises to 0.4174 —
            // and the worst muscle MOVES from `gasmed_r` to `psoas_l` — so the
            // primary clause CROSSES its bar and now fails there too, while
            // video_015 IMPROVES to 0.1202 and clears both clauses.
            //
            // THIS IS THE PRE-REGISTERED DOWNSIDE ACTUALLY HAPPENING. STATUS's
            // round-16 header said, before the regeneration ran, that the six
            // clauses passing at 4 s might fail at 8 s and that a net-worse
            // outcome would be reported as net-worse. `g8PrimaryExcursion` is
            // untouched at 0.30 and is asserted below where it is read; what
            // changed is the measurement. The direction is recorded per clip
            // rather than smoothed into one verdict, in the same idiom G2(c)
            // already uses for a clause that holds on one clip only.
            if pin.worst <= RegisteredBar.g8PrimaryExcursion {
                XCTAssertLessThanOrEqual(pin.worst, RegisteredBar.g8PrimaryExcursion,
                    "G8(a) primary clause is recorded as PASSED on \(clip)")
            } else {
                XCTAssertGreaterThan(pin.worst, RegisteredBar.g8PrimaryExcursion,
                    "G8(a) primary clause is recorded as FAILED on \(clip)")
                recordFailedGate("G8(a)-primary", clip: clip,
                                 measured: String(format: "worst=%.6f on %@ over %d muscles",
                                                  pin.worst, pin.worstMuscle, pin.muscles),
                                 bar: "every muscle <= \(RegisteredBar.g8PrimaryExcursion)",
                                 why: "the 8 s window's longer trace turns what was a passing "
                                      + "0.2971 into 0.4174 and moves the worst muscle from "
                                      + "gasmed_r to psoas_l; a clause that passed at 4 s now "
                                      + "fails, which the round-16 pre-registration named as a "
                                      + "possible outcome BEFORE this ran; "
                                      + "bbox_source=macos_vision INTERIM")
            }
            let secondaryAllowance = Int((RegisteredBar.g8SecondaryFraction
                                          * Double(pin.muscles)).rounded(.down))
            if over <= secondaryAllowance {
                print("MODE-VERDICT gate=G8(a) clip=\(clip) outcome=PASS_NON_VACUOUS "
                      + "muscles=\(pin.muscles) "
                      + String(format: "worst=%.6f on %@", pin.worst, pin.worstMuscle)
                      + " over20=\(over) allowance=\(secondaryAllowance) "
                      + "bbox_source=macos_vision INTERIM")
            } else {
                XCTAssertGreaterThan(over, secondaryAllowance,
                                     "G8(a) is recorded as FAILED on \(clip)")
                recordFailedGate("G8(a)", clip: clip,
                                 measured: "muscles=\(pin.muscles) "
                                           + String(format: "worst=%.6f on %@ (<= %.2f primary, "
                                                    + "so the PRIMARY clause holds)",
                                                    pin.worst, pin.worstMuscle,
                                                    RegisteredBar.g8PrimaryExcursion)
                                           + " over20=\(over) of \(pin.muscles) "
                                           + "allowance=\(secondaryAllowance)",
                                 bar: "all <= \(RegisteredBar.g8PrimaryExcursion), at most "
                                      + "\(RegisteredBar.g8SecondaryFraction) of them over "
                                      + "\(RegisteredBar.g8SecondaryExcursion)",
                                 why: "the SECONDARY allowance is blown: \(over) of \(pin.muscles) "
                                      + "scored muscles carry more than 20 % of their own range as "
                                      + "end-to-end trend; bbox_source=macos_vision INTERIM")
            }
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
